"""
    Module to handle the computer vision - convert hand positions into end-effector targets
    Author: Neo Vorsatz
    Last updated: 21 October 2026
"""

# IMPORTS ================================

# External
from math import log
import time
import cv2
import mediapipe as mp
import numpy as np
mptv = mp.tasks.vision
from math import sqrt

# Internal
import modules.data_collection as dc

# TYPES ================================

_HandLandmarkerResult = mptv.HandLandmarkerResult
_Coord = list[float]
_NDArray = np.typing.NDArray

# CONSTANTS ================================

# Benchmarking
_BENCHMARK = True # make this `False` to omit code that is for benchmarking
_HD_FETCH_FRAME = "fetch frame (ms)"
_HD_LANDMARK = "landmark (ms)"
_HD_CALC_TARGET = "calc target (ms)"

# Low pass filter
_WINDOW_SIZE = 10

# Chosen camera
_CAMERA_INDEX = 0

# Filepath to model parameters
_MODEL_PATH = "mediapipe_models/hand_landmarker.task"

# Physical constraints
_BASE_RADIUS = 100 # distance to stay away form the origin

# Callibration and mapping
_SAMPLES = 10 # number of samples for one entry in callibration
_SLAVE_SPACE_X = 300 # maximum x distance in slave space
_SLAVE_SPACE_Y = 300 # maximum y distance in slave space
_SLAVE_SPACE_Z = 300 # maximum z distance in slave space
_GRASP_PAD = 0.2 # how much larger the grasp needs to be to update the bounds

# Landmarks
_LM_BASE = 0
_LM_INDEX_BASE = 5
_LM_MIDDLE_FINGER = 12 # consider using 8 (tip of index finger)
_LM_THUMB = 4
_LM_END_EFFECTOR = -1 # a fictitious landmark that is the midpoint between the thumb and middle finger

# Vector components
_X = 0
_Y = 1
_Z = 2

# VARIABLES ================================

# Flags
_active:bool = False

# Runtime processors
_video_stream = None
_hand_landmarker = None

# Configuration
_start_time:float = None

# Latest data
_latest_result = None
_key_press:int = 255 # no key pressed

# Callibration
_offset:_NDArray = None # offset vector
_transform:_NDArray = None # transformation matrix
_grasp_max = 40 # maximum distance between grasp
_grasp_min = 30 # minimum distance between grasp

# Low pass filter
_past_targets:dict[str,list[_Coord]] = {
    "pos":[], "dir":[], "rot":[]
}

# HELPER FUNCTIONS ================================

def _handle_result(result:_HandLandmarkerResult, output_image:mp.Image, timestamp_ms:int):
    """
    Called asynchronously when MediaPipe finishes analysing a frame
    """
    global _latest_result, _start_time
    _latest_result = result

    # Benchmarking
    if _BENCHMARK:
        bench_delta = (time.monotonic()-_start_time)*1000 - timestamp_ms
        dc.record(bench_delta, _HD_LANDMARK)

def _proxy_depth(result:_HandLandmarkerResult) -> float:
    """
    Gets the depth of the hand, using the apparent size as a proxy for depth

    Args:
        result (_HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
    Returns:
        float: The depth away from the camera due to apparent size
    """
    # Calculate size
    hand = result.hand_landmarks[0]
    base_xy:_NDArray = np.array([hand[_LM_BASE].x, hand[_LM_BASE].y])
    index_base_xy:_NDArray = np.array([hand[_LM_INDEX_BASE].x, hand[_LM_INDEX_BASE].y])
    size = np.linalg.norm(np.subtract(index_base_xy, base_xy))

    # Calculate depth
    return 1/size

def _get_landmark_coord(result:_HandLandmarkerResult, landmark_index:int) -> _Coord | None:
    """
    Gets the x, y and z coordinates of a landmark, from the first hand detected

    Args:
        result (_HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
        landmark_index (int): The index of the landmark
    Returns:
        _Coord | None: A list of the x, y and z coordinates of the landmark, or None if there is no hand detected
    """
    # Initialize coordinates
    coord:_Coord = [None, None, None]

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
    else:
        # Validate result
        if result is None or not result.hand_landmarks:
            return None

        # Get landmarks
        hand = result.hand_landmarks[0]
        coord[_X] = hand[landmark_index].x
        coord[_Y] = hand[landmark_index].y
        coord[_Z] = hand[landmark_index].z

    # Add proxy depth
    coord[_Z] += _proxy_depth(result)
    return coord

def _draw_all_landmarks_on_image(frame, result:_HandLandmarkerResult):
    """
    Draws hand landmarks and connections onto a BGR numpy frame, in-place

    Args:
        frame: The video frame to draw on
        result (_HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
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

def _draw_landmarks(frame, result:_HandLandmarkerResult):
    """
    Draws the relevant landmarks and connection on the frame, in-place
    
    Args:
        frame: The video frame to draw on
        result (_HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
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
    cv2.circle(frame, middle_finger_pt, 4, (0, 255, 255), -1) # middle finger
    cv2.circle(frame, thumb_pt, 4, (0, 0, 255), -1) # thumb
    cv2.circle(frame, end_effector_pt, 4, (255, 0, 0), -1) # end effector

def _wait_for_key(key:str=""):
    """
    Continuously updates the video stream and landmarks until a key is pressed.
    This function is blocking

    Args:
        key (str): The key to wait for
    """
    if len(key)==1:
        compare = ord(key)
    else:
        compare = 255 # no key
    update_cv()
    while get_key_press()!=compare:
        update_cv()

def _avg_coords(coords:list[_Coord | None]) -> _Coord:
    """
    Averages a list of coordinates

    Args:
        coords (list[_Coord | None]): A list of coordinates
    Returns:
        _Coord: The averaged coordinate
    Raises:
        Exception: If the list is empty or contains no valid coordinates
    """
    # Initialize
    elements = 0
    average = [0, 0, 0]

    # Sum all valid elements
    for coord in coords:
        if coord is not None:
            elements += 1
            for i in range(3):
                average[i] += coord[i]

    # Validate
    if elements==0:
        raise ValueError("No valid coordinates to average")

    # Divide by number of elements
    for i in range(3):
        average[i] /= elements
    return average

def _get_avg_end_effector() -> _Coord:
    """
    Waits for key-press activation,
    samples the computer vision multiple times,
    and averages the end-effector coordinate

    Returns:
        _Coord: Averaged end-effector coordinate
    Raises:
        ValueError: If no hand is in the camera's view
    """
    # Global variables
    global _latest_result

    # Wait for key trigger
    _wait_for_key()
    _wait_for_key(" ")

    # Get samples
    coords:list[_Coord] = []
    for i in range(_SAMPLES):
        update_cv()
        coords.append(_get_landmark_coord(_latest_result, _LM_END_EFFECTOR))
        time.sleep(0.001) # give the asynchronous `_hand_landmarker` time to process

    # Validate
    try:
        avg_coord = _avg_coords(coords)
    except ValueError:
        raise ValueError("Hand is not visible for callibration")

    # Return average
    return avg_coord

def _normalize(vector:_Coord) -> _Coord:
    """
    Normalises a vector

    Args:
        vector (_Coord): The vector to normalise
    Returns:
        _Coord: The normalized version
    """
    magnitude = sqrt(vector[_X]**2 + vector[_Y]**2 + vector[_Z]**2)
    if magnitude != 0:
        norm = []
        for i in range(3):
            norm.append(vector[i] / magnitude)
        return norm
    else:
        return vector.copy()

def _get_transformed_coord(landmarks:_HandLandmarkerResult, landmark_index:int) -> _NDArray | None:
    """
    Gets the corrected/callibrated x, y and z coordinates of a landmark

    Args:
        result (_HandLandmarkerResult): The MediaPipe Hand Landmark Detection result
        landmark_index (int): The index of the landmark
    Returns:
        _NDArray: A list of the corrected/callibrated x, y and z coordinates of the landmark
    """
    # Global variables
    global _offset, _transform

    # Get raw coordinates
    coord = _get_landmark_coord(landmarks, landmark_index)
    if coord is None:
        return None
    coord = np.array(coord)

    # Apply offset
    coord = np.add(coord, _offset)

    # Apply transformation
    coord = np.dot(_transform, coord)

    # Return
    return coord

def _low_pass(coord:_Coord, key:str) -> _Coord:
    """
    Performs a low pass filter on the coordinate, assuming the coordinate is valid

    Args:
        coord (_Coord): The next coordinate
        key (str): The low pass filter buffer to use
    Returns:
        _Coord: The low pass filtered coordinate
    """
    # Global variables
    global _past_targets

    # Append and truncate the buffer
    _past_targets[key].append(coord)
    if len(_past_targets[key]) > _WINDOW_SIZE:
        _past_targets[key].pop(0)

    # Return average
    return _avg_coords(_past_targets[key])

def _clamp_position(pos:_Coord) -> _Coord:
    """
    Clamps a position (in-place editing) into something more reasonable by:
    - preventing negative y positions (backwards workspace)
    - preventing negative z positions (below workspace)
    - preventing positions directly on the base (physically unreachable)

    Args:
        pos (_Coord): The position to clamp
    Returns:
        _Coord: The clamped position
    """
    # Prevent negative y
    if pos[_Y] < 0.0:
        pos[_Y] = 0.0

    # Prevent negative z
    if pos[_Z] < 0.0:
        pos[_Z] = 0.0;

    # Prevent coordinates directly on the base
    r = sqrt(pos[_X]**2 + pos[_Y]**2 + pos[_Z]**2) # radius from the origin (centre of the base)
    if r < _BASE_RADIUS: # if the distance is too small
        if r==0.0: # if the position is at the origin
            pos[_Y] = _BASE_RADIUS # set the default position to "forward"
        else:
            scale = _BASE_RADIUS / r
            for i in range(3):
                pos[i] *= scale

    # Return
    return pos

def _clamp_direction(dir:_Coord, pos:_Coord) -> _Coord:
    """
    Clamps a direction (in-place editing) into something more reasonable by:
    - realigning the direction to what's achievable

    Args:
        dir (_Coord): The direction to clamp
        pos (_Coord): The position of the end effector
    Returns:
        _Coord: The clamped direction
    """
    # Project dir onto place defined by pos and the z axis
    cross = [ # cross product between position and z unit vector
        pos[_Y]*(1)-pos[_Z]*(0),
        pos[_X]*(1)-pos[_Z]*(0),
        pos[_X]*(0)-pos[_Y]*(0)
    ]
    diff = dir[_X]*cross[_X] + dir[_Y]*cross[_Y] + dir[_Z]*cross[_Z] # how much dir protrudes out of the pos-z plane
    for i in range(3):
        dir[i] -= diff*cross[i] # project dir onto the pos-z plane

    # Return
    return dir

# API FUNCTIONS ================================

def init_cv():
    """
    Starts the OpenCV video stream and
    creates a MediaPipe landmarker that handles asynchronous hand landmark detection 

    Args:
        exit_key (str): A single character string representing the key that kills the computer vision pipeline
    """
    # Global variables
    global _active, _video_stream, _hand_landmarker, _start_time, _latest_result

    # Note that the computer vision is active
    _active = True

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

    # Benchmarking
    if _BENCHMARK:
        dc.add_heading(_HD_FETCH_FRAME)
        dc.add_heading(_HD_LANDMARK)
        dc.add_heading(_HD_CALC_TARGET)

    # Record start time for timestamps
    _start_time = time.monotonic()

def calibrate_cv():
    """
    Capture critical positions to create a mapping from hand poses to end-effector poses.
    This function must be called after `init_cv()` and before any instance of `get_target()`

    Raises:
        ValueError: If the hand was not visible in one of the stages of calibration
    """
    # Global variables
    global _latest_result, _offset, _transform

    # Get critical coordinates
    print("Callibration: Centre")
    centre_coord = _get_avg_end_effector()
    print("Callibration: Right")
    right_coord = _get_avg_end_effector()
    print("Callibration: Forward")
    forward_coord = _get_avg_end_effector()
    print("Callibration: Up")
    up_coord = _get_avg_end_effector()

    # Create groupings
    boundary_coords = [right_coord, forward_coord, up_coord] # group the boundary coordinates
    boundary_dist = [_SLAVE_SPACE_X, _SLAVE_SPACE_Y, _SLAVE_SPACE_Z] # group maximum distances

    # Use the centre to determine the offsets
    _offset = -1*np.array(centre_coord)

    # Create transformation matrix from slave space to master space
    array_2d = []
    for row in range(3):
        array_2d.append([])
        for col in range(3):
            element = (boundary_coords[col][row]-centre_coord[row])/boundary_dist[col]
            array_2d[row].append(element)
    slave_to_master = np.array(array_2d)

    # Convert transformation from slave to master into master to slave
    _transform = np.linalg.pinv(slave_to_master)

    # Confirm success
    print("Successful callibration")

def update_cv() -> bool:
    """
    Updates the computer vision's pose estimation and video feedback

    Returns:
        Whether or not the computer vision is active
    """
    # Global variables
    global _active, _video_stream, _latest_result, _hand_landmarker, _start_time, _key_press

    # Benchmarking
    if _BENCHMARK:
        bench_start = time.perf_counter()

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
    frame_timestamp_ms = int((time.monotonic()-_start_time)*1000)

    # Give image to Hand Landmarker
    _hand_landmarker.detect_async(mp_image, frame_timestamp_ms)

    # Display the frame
    _draw_landmarks(frame, _latest_result)
    # _draw_all_landmarks_on_image(frame, _latest_result)
    cv2.imshow("Hand Landmarks", frame)

    # Benchmarking
    if _BENCHMARK:
        bench_delta = (time.perf_counter()-bench_start)*1000
        dc.record(bench_delta, _HD_FETCH_FRAME)

    # Get key press
    _key_press = cv2.waitKey(1) & 0xFF

    # Return success
    return True

def get_key_press() -> int:
    """
    Returns:
        The ordinal value of the latest key press
    """
    global _key_press
    return _key_press

def get_target() -> tuple[list[list[float]], float] | tuple[None, None]:
    """
    Converts the latest HandLandmarkerResult into a target

    Returns:
        (list[list[float]], float): The target position, direction and rotation
    """
    # Global variables
    global _grasp_max, _grasp_min

    # Benchmarking
    if _BENCHMARK:
        bench_start = time.perf_counter()

    # Get coordinates
    base = _get_transformed_coord(_latest_result, _LM_BASE)
    middle_finger = _get_transformed_coord(_latest_result, _LM_MIDDLE_FINGER)
    thumb = _get_transformed_coord(_latest_result, _LM_THUMB)
    end_effector = _get_transformed_coord(_latest_result, _LM_END_EFFECTOR)

    # Check that coordinates are valid
    if base is None:
        return (None, None)

    # Set position
    position = end_effector.tolist()
    position[_X] = 0.0 # REJECT DEPTH
    position = _clamp_position(position)

    # Set direction
    direction = np.subtract(end_effector, base).tolist()
    direction[_X] = 0.0 # REJECT DEPTH
    direction = _normalize(direction)

    # Get vector from thumb to middle finger
    thm_to_mid = np.subtract(middle_finger, thumb)
    if thm_to_mid[_X] < 0:
        thm_to_mid[_X] *= -1 # correct flaw in model training
    # print("Claw tips | thumb:", thumb[0], " | middle:", middle_finger[0]) # FEEDBACK

    # Set rotation
    rotation = _normalize(thm_to_mid.tolist())

    # Update grasp bounds
    grasp = np.linalg.norm(thm_to_mid)
    if grasp > _grasp_max*(1.0+_GRASP_PAD):
        _grasp_max = grasp
    elif grasp < _grasp_min*(1.0-_GRASP_PAD):
        _grasp_min = grasp

    # Calculate openness
    openness = 73 + grasp*(10-73)/(_grasp_max-_grasp_min)
    print("Claw | norm:", np.linalg.norm(thm_to_mid), end="") # FEEDBACK
    print(" | openness:", openness) # FEEDBACK

    # Benchmarking
    if _BENCHMARK:
        bench_delta = (time.perf_counter()-bench_start)*1000
        dc.record(bench_delta, _HD_CALC_TARGET)

    # Return
    return ([position, direction, rotation], openness)

def cleanup_cv():
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

# MAIN ================================

if __name__ == "__main__":
    # Imports
    import sys

    # Initialization
    init_cv()

    try:
        # Calibrate
        try:
            calibrate_cv()
        except (BaseException, ValueError, KeyboardInterrupt) as e:
            cleanup_cv()
            print(e)
            sys.exit() # exit with error

        # Continuously update computer vision
        while get_key_press() != ord("q"):
            if not update_cv():
                break
            target, openness = get_target()
            if target is None:
                print("No hand")
                pass
            else:
                print("End-effector:", round(target[0][_X]), round(target[0][_Y]), round(target[0][_Z])) # print position of end-effector
    except BaseException as e:
        print(e)

    # Free memory and processes
    cleanup_cv()
