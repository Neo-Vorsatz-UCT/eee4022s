# IMPORTS ================================

# External
import time
import cv2
import mediapipe as mp
mptv = mp.tasks.vision

# TYPES ================================

HandLandmarkerResult = mptv.HandLandmarkerResult

# CONSTANTS ================================

CAMERA_INDEX = 0
MODEL_PATH = "mediapipe_models/hand_landmarker.task"

# HELPER FUNCTIONS ================================

def handle_result(result:HandLandmarkerResult, output_image:mp.Image, timestamp_ms:int):
    """
    Called asynchronously when MediaPipe finishes analysing a frame
    """
    global latest_result
    latest_result = result

def draw_all_landmarks_on_image(frame, result:HandLandmarkerResult):
    """
    Draws hand landmarks and connections onto a BGR numpy frame
    """
    # Define all connections
    HAND_CONNECTIONS = [
        (0, 1), (1, 2), (2, 3), (3, 4),        # thumb
        (0, 5), (5, 6), (6, 7), (7, 8),        # index
        (5, 9), (9, 10), (10, 11), (11, 12),   # middle
        (9, 13), (13, 14), (14, 15), (15, 16), # ring
        (13, 17), (17, 18), (18, 19), (19, 20),# pinky
        (0, 17),                               # palm base
    ]

    # Validate result
    if result is None or not result.hand_landmarks:
        return frame

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

def draw_landmarks(frame, result:HandLandmarkerResult):
    """
    Draws the relevant landmarks and connection on the frame
    """
    # Landmarks
    LM_BASE = 0
    LM_MIDDLE_FINGER = 12
    LM_THUMB = 4

    # Validate result
    if result is None or not result.hand_landmarks:
        return frame

    # Function to convert landmarks to points on a screen
    def landmark_to_point(index:int) -> tuple[int,int]:
        return (int(width*hand[index].x), int(height*hand[index].y))

    height, width, _ = frame.shape
    for hand in result.hand_landmarks: # for each hand
        # Calculate coordinates on screen
        base_pt = landmark_to_point(LM_BASE)
        middle_finger_pt = landmark_to_point(LM_MIDDLE_FINGER)
        thumb_pt = landmark_to_point(LM_THUMB)
        end_effector_pt = (int((thumb_pt[0]+middle_finger_pt[0])/2), int((thumb_pt[1]+middle_finger_pt[1])/2))

        # Draw connections
        cv2.line(frame, middle_finger_pt, thumb_pt, (0, 255, 0), 2) # openness
        cv2.line(frame, base_pt, end_effector_pt, (0, 255, 0), 2) # placement

        # Draw landmarks
        cv2.circle(frame, base_pt, 4, (0, 0, 255), -1) # base
        cv2.circle(frame, middle_finger_pt, 4, (0, 0, 255), -1) # middle finger
        cv2.circle(frame, thumb_pt, 4, (0, 0, 255), -1) # thumb
        cv2.circle(frame, end_effector_pt, 4, (255, 0, 0), -1) # end effector

# SETUP ================================

# Camera
video_stream = cv2.VideoCapture(CAMERA_INDEX)

# Latest completed result from MediaPipe
latest_result = None

# Create a hand landmarker instance for a live stream
options = mptv.HandLandmarkerOptions(
    base_options=mp.tasks.BaseOptions(model_asset_path=MODEL_PATH),
    num_hands=1,
    running_mode=mptv.RunningMode.LIVE_STREAM,
    result_callback=handle_result,
)
hand_landmarker = mptv.HandLandmarker.create_from_options(options)

# Record start time for timestamps
start_time = time.time()

# LOOP ================================

while True:
    # Read frame
    success, frame = video_stream.read()
    if not success or frame is None:
        continue

    # Convert BGR frame to RGB MediaPipe image
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    # Generate timestamp
    frame_timestamp_ms = int((time.time()-start_time)*1000)

    # Give image to Hand Landmarker
    hand_landmarker.detect_async(mp_image, frame_timestamp_ms)

    # Display the frame
    draw_landmarks(frame, latest_result)
    # draw_all_landmarks_on_image(frame, latest_result)
    cv2.imshow("Hand Landmarks", frame)

    # Check exit condition
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# CLEANUP ================================

hand_landmarker.close()
video_stream.release()
cv2.destroyAllWindows()
