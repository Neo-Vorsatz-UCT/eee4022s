"""
    Module to handle the computer vision - convert hand positions into end-effector targets
    Author: Neo Vorsatz
    Last updated: 7 October 2026
"""

# IMPORTS ================================

# External
from math import sqrt
import time
import cv2
import mediapipe as mp
import numpy as np
mptv = mp.tasks.vision

# TYPES ================================

HandLandmarkerResult = mptv.HandLandmarkerResult
_NDArray = np.typing._NDArray

class PoseHandler():
    """
    A class to handle a hand's pose: position, orientation and openness
    """
    SLAVE_SPACE_X = 300 # maximum x distance in slave space
    SLAVE_SPACE_Y = 300 # maximum y distance in slave space
    SLAVE_SPACE_Z = 300 # maximum z distance in slave space
    
    def __init__(self):
        # Callibration
        self.offset:_NDArray = np.array([0, 0, 0]) # origin offset
        self.transform:_NDArray = np.identity(3) # identity matrix

        # Pose
        self.position:list[float] = [0, 0, 0] # position of the hand
        self.direction:list[float] = [1, 0, 0] # direction of the hand, normalised
        self.rotation:list[float] = [0, 1, 0] # auxillary perpendicular axis for rotation, normalised
        self.openness:float = 1 # distance from pinching action

    def _normalize(self, vector:_NDArray):
        """
        Normalises a vector

        Args:
            vector (_NDArray): The vector to normalise
        Returns:
            _NDArray: The normalized version
        """
        magnitude = np.linalg.norm(vector)
        if magnitude != 0:
            return vector / magnitude
        else:
            return vector.copy()

    def callibrate(self, centre:HandLandmarkerResult, right:HandLandmarkerResult, forward:HandLandmarkerResult, up:HandLandmarkerResult) -> bool:
        """
        Sets the offsets and scaling factors based off how readings correspond to known positions.
        This function uses the convention that "right" should be X, "forward" should be Y, and "up" should be Z,
        which aligns with the inverse kinematics solver

        Args:
            centre (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result for the centre of the workspace
            right (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result for the right of the workspace
            forward (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result for the front of the workspace
            up (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result for the top of the workspace
        Returns:
           bool: Whether the callibration was successful or not
        """
        # Get coordinates
        centre_coord = _get_landmark_coord(centre, _LM_END_EFFECTOR);
        right_coord = _get_landmark_coord(forward, _LM_END_EFFECTOR);
        forward_coord = _get_landmark_coord(forward, _LM_END_EFFECTOR);
        up_coord = _get_landmark_coord(up, _LM_END_EFFECTOR);

        # Create groupings
        boundary_coords = [right_coord, forward_coord, up_coord] # group the boundary coordinates
        boundary_dist = [self.SLAVE_SPACE_X, self.SLAVE_SPACE_Y, self.SLAVE_SPACE_Z] # group maximum distances

        # Use the centre to determine the offsets
        if centre_coord is None:
            return False
        self.offset = -1*np.array(centre_coord)

        # Create transformation matrix from slave space to master space
        array_2d = []
        for row in range(3):
            array_2d.append([])
            for col in range(3):
                element = (boundary_coords[col][row]-centre_coord[row])/boundary_dist[col]
                array_2d[row].append(element)
        slave_to_master = np.array(array_2d)

        # Convert transformation from slave to master into master to slave
        self.transform = np.linalg.pinv(slave_to_master)

        # Note that the process was successful
        return True

    def _get_transformed_coord(self, landmarks:HandLandmarkerResult, landmark_index:int) -> _NDArray | None:
        """
        Gets the corrected/callibrated x, y and z coordinates of a landmark
    
        Args:
            result (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
            landmark_index (int): The index of the landmark
        Returns:
            _NDArray: A list of the corrected/callibrated x, y and z coordinates of the landmark
        """
        # Get raw coordinates
        coords = _get_landmark_coord(landmarks, landmark_index)
        if coords is None:
            return None
        coords = np.array(coords)

        # Apply offset
        coords = np.add(coords, self.offset)

        # Apply transformation
        return np.dot(self.transform, coords)

    def set_pose(self, landmarks:HandLandmarkerResult):
        """
        Sets the position of the pose

        Args:
            landmarks (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
        """
        # Get coordinates
        base = self._get_transformed_coord(landmarks, _LM_BASE)
        middle_finger = self._get_transformed_coord(landmarks, _LM_MIDDLE_FINGER)
        thumb = self._get_transformed_coord(landmarks, _LM_THUMB)
        end_effector = self._get_transformed_coord(landmarks, _LM_END_EFFECTOR)

        # Check that coordinates are valid
        if base is None:
            return

        # Set position
        self.position = end_effector.tolist()

        # Set direction
        self.direction = self._normalize(np.subtract(end_effector, base)).to_list()

        # Get vector from thumb to middle finger
        thm_to_mid = np.subtract(middle_finger, thumb)

        # Set rotation
        self.rotation = self._normalize(thm_to_mid).to_list()

        # Set openness
        self.openness = np.linalg.norm(thm_to_mid)

# CONSTANTS ================================

# Chosen camera
_CAMERA_INDEX = 0

# Filepath to model parameters
_MODEL_PATH = "mediapipe_models/hand_landmarker.task"

# Landmarks
_LM_BASE = 0
_LM_MIDDLE_FINGER = 12
_LM_THUMB = 4
_LM_END_EFFECTOR = -1 # a fictious landmark that is the midpoint between the thumb and middle finger

# Vector components
_X = 0
_Y = 1
_Z = 2

# VARIABLES ================================

_active:bool = False
_exit_key = None
_video_stream = None
_latest_result = None
_hand_landmarker = None
_start_time:float = None

# HELPER FUNCTIONS ================================

def _handle_result(result:HandLandmarkerResult, output_image:mp.Image, timestamp_ms:int):
    """
    Called asynchronously when MediaPipe finishes analysing a frame
    """
    global _latest_result
    _latest_result = result

def _get_landmark_coord(result:HandLandmarkerResult, landmark_index:int) -> list[float] | None:
    """
    Gets the x, y and z coordinates of a landmark, from the first hand detected

    Args:
        result (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
        landmark_index (int): The index of the landmark
    Returns:
        list[float]: A list of the x, y and z coordinates of the landmark
    """
    # Initialize coordinates
    coord:list[float] = [None, None, None]

    # If the desired landmark is the fictional end effector
    if landmark_index == _LM_END_EFFECTOR:
        # Get thumb and middle finger
        thumb_coord = _get_landmark_coord(result, _LM_THUMB)
        middle_finger_coord = _get_landmark_coord(result, _LM_MIDDLE_FINGER)

        # Check for None values
        if (thumb_coord is None) or (middle_finger_coord is None):
            return None

        # Average the coordinates
        for i in range(3):
            coord[i] = (thumb_coord[i]+middle_finger_coord[i])/2
        return coord

    # Validate result
    if result is None or not result.hand_landmarks:
        return None

    # Get landmarks
    hand = result.hand_landmarks[0]
    coord[_X] = hand[landmark_index].x
    coord[_Y] = hand[landmark_index].y
    coord[_Z] = hand[landmark_index].z
    return coord

def _draw_all_landmarks_on_image(frame, result:HandLandmarkerResult):
    """
    Draws hand landmarks and connections onto a BGR numpy frame, in-place

    Args:
        frame: The video frame to draw on
        result (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
    Returns:
        The modified frame
    """
    # Validate result
    if result is None or not result.hand_landmarks:
        return frame
    
    # Define all connections
    HAND_CONNECTIONS = [
        (0, 1), (1, 2), (2, 3), (3, 4), # thumb
        (0, 5), (5, 6), (6, 7), (7, 8), # index
        (5, 9), (9, 10), (10, 11), (11, 12), # middle
        (9, 13), (13, 14), (14, 15), (15, 16), # ring
        (13, 17), (17, 18), (18, 19), (19, 20), # pinky
        (0, 17), # palm base
    ]

    # Draw
    height, width, _ = frame.shape
    for hand_landmarks in result.hand_landmarks:
        # Landmarks are normalized (0-1), so they must be converted to pixel coordinates
        points = [(int(lm.x * width), int(lm.y * height)) for lm in hand_landmarks]
        # Draw connections
        for start_idx, end_idx in HAND_CONNECTIONS:
            cv2.line(frame, points[start_idx], points[end_idx], (0, 255, 0), 2)
        # Draw landmarks
        for point in points:
            cv2.circle(frame, point, 4, (0, 0, 255), -1)
    
    # Return
    return frame

def _draw_landmarks(frame, result:HandLandmarkerResult):
    """
    Draws the relevant landmarks and connection on the frame, in-place
    
    Args:
        frame: The video frame to draw on
        result (HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
    Returns:
        The modified frame
    """
    # Validate result
    if result is None or not result.hand_landmarks:
        return frame

    # Function to convert landmarks to points on a screen
    height, width, _ = frame.shape
    def landmark_to_point(index:int) -> tuple[int,int]:
        coord = _get_landmark_coord(result, index)
        return (int(width*coord[_X]), int(height*coord[_Y]))

    # Calculate coordinates on screen
    base_pt = landmark_to_point(_LM_BASE)
    middle_finger_pt = landmark_to_point(_LM_MIDDLE_FINGER)
    thumb_pt = landmark_to_point(_LM_THUMB)
    end_effector_pt = landmark_to_point(_LM_END_EFFECTOR)

    # Draw connections
    cv2.line(frame, middle_finger_pt, thumb_pt, (0, 255, 0), 2) # openness
    cv2.line(frame, base_pt, end_effector_pt, (0, 255, 0), 2) # placement

    # Draw landmarks
    cv2.circle(frame, base_pt, 4, (0, 0, 255), -1) # base
    cv2.circle(frame, middle_finger_pt, 4, (0, 0, 255), -1) # middle finger
    cv2.circle(frame, thumb_pt, 4, (0, 0, 255), -1) # thumb
    cv2.circle(frame, end_effector_pt, 4, (255, 0, 0), -1) # end effector

def _cleanup():
    """
    Clean up any resources used by the computer vision module
    """
    # Global variables
    global _active, _video_stream, _hand_landmarker

    # Note that the computer vision is inactive
    _active = False

    # Cleanup
    _hand_landmarker.close()
    _video_stream.release()
    cv2.destroyAllWindows()

# API FUNCTIONS ================================

def init_cv(exit_key:str="q"):
    """
    Starts the OpenCV video stream and
    creates a MediaPipe landmarker that handles asynchronous hand landmark detection 

    Args:
        exit_key (str): A single character string representing the key that kills the computer vision pipeline
    """
    # Global variables
    global _active, _exit_key, _video_stream, _latest_result, _hand_landmarker, _start_time

    # Note that the computer vision is active
    _active = True

    # Exit key
    if len(exit_key)==1:
        _exit_key = exit_key
    else:
        _exit_key = "q"

    # Camera
    _video_stream = cv2.VideoCapture(_CAMERA_INDEX)

    # Latest completed result from MediaPipe
    _latest_result = None

    # Create a hand landmarker instance for a live stream
    options = mptv.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=_MODEL_PATH),
        num_hands=1,
        running_mode=mptv.RunningMode.LIVE_STREAM,
        result_callback=_handle_result,
    )
    _hand_landmarker = mptv.HandLandmarker.create_from_options(options)

    # Record start time for timestamps
    _start_time = time.time()

def get_landmarks() -> HandLandmarkerResult:
    """
    Returns the latest valid landmark results.

    Returns:
        HandLandmarkerResult: The latest valid landmark results
    """
    global _latest_result
    return _latest_result

def update_cv() -> bool:
    """
    Updates the computer vision's pose estimation and video feedback

    Returns:
        Whether or not the computer vision is active
    """
    # Global variables
    global _active, _video_stream, _latest_result, _hand_landmarker, _start_time

    # Check activity
    if not _active:
        return False

    # Read frame
    success, frame = _video_stream.read()
    if not success or frame is None:
        return True

    # Convert BGR frame to RGB MediaPipe image
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    # Generate timestamp
    frame_timestamp_ms = int((time.time()-_start_time)*1000)

    # Give image to Hand Landmarker
    _hand_landmarker.detect_async(mp_image, frame_timestamp_ms)

    # Display the frame
    _draw_landmarks(frame, _latest_result)
    # _draw_all_landmarks_on_image(frame, _latest_result)
    cv2.imshow("Hand Landmarks", frame)

    # Check exit condition
    if cv2.waitKey(1) & 0xFF == ord(_exit_key):
        _cleanup()
        return False
    return True

# MAIN ================================

if __name__ == "__main__":
    # Initialization
    init_cv()

    # Continuously update computer vision
    while update_cv():
        pass
