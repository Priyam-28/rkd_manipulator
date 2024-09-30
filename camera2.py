import cv2
import serial
import mediapipe as mp
import numpy as np

# Configurations
debug = False  # Set to True for debugging without Arduino
cam_source = 0  # Set to 0 for the default webcam

if not debug:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Set to your serial port
    ser.timeout = 1

# Servo angle ranges
base_min, base_max = 0, 180
left_servo_min, left_servo_max = 0, 180
right_servo_min, right_servo_max = 0, 180
claw_open, claw_close = 0, 180

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Utility functions for mapping and clamping
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
map_range = lambda x, in_min, in_max, out_min, out_max: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function to calculate angle between three points
def calculate_angle(a, b, c):
    a = np.array(a)  # First point (elbow)
    b = np.array(b)  # Middle point (shoulder)
    c = np.array(c)  # Last point (wrist)

    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)

    if angle > 180.0:
        angle = 360 - angle

    return angle

# Initialize webcam
cap = cv2.VideoCapture(cam_source)

# Main loop for pose detection and servo control
while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    # Flip the image for selfie-view and convert to RGB
    image = cv2.flip(image, 1)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)

    # Default servo angles
    base_angle = 90
    left_servo_angle = 90
    right_servo_angle = 90
    claw_angle = claw_open

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # Extract required landmarks for calculations (elbow, shoulder, wrist, etc.)
        left_shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, 
                         landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
        left_elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, 
                      landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
        left_wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, 
                      landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
        
        right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, 
                          landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
        right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, 
                       landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
        right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, 
                       landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

        # Calculate the angle for the base motor using left elbow and shoulder
        left_elbow_angle = calculate_angle(left_shoulder, left_elbow, left_wrist)
        base_angle = map_range(left_elbow_angle, 70, 160, base_min, base_max)  # Adjust ranges as per your needs
        base_angle = clamp(base_angle, base_min, base_max)

        # Calculate wrist angles for left and right servo control
        left_wrist_angle = calculate_angle(left_elbow, left_wrist, [left_wrist[0] + 1, left_wrist[1]])  # Angle relative to x-axis
        left_servo_angle = map_range(left_wrist_angle, 0, 180, left_servo_min, left_servo_max)
        left_servo_angle = clamp(left_servo_angle, left_servo_min, left_servo_max)

        right_wrist_angle = calculate_angle(right_elbow, right_wrist, [right_wrist[0] + 1, right_wrist[1]])
        right_servo_angle = map_range(right_wrist_angle, 0, 180, right_servo_min, right_servo_max)
        right_servo_angle = clamp(right_servo_angle, right_servo_min, right_servo_max)

        # Claw control based on open/closed hand (approximated by distance between thumb and fingers)
        left_thumb_tip = landmarks[mp_pose.PoseLandmark.LEFT_THUMB_TIP.value]
        left_index_tip = landmarks[mp_pose.PoseLandmark.LEFT_INDEX_FINGER_TIP.value]
        thumb_index_distance = np.linalg.norm(np.array([left_thumb_tip.x, left_thumb_tip.y]) - np.array([left_index_tip.x, left_index_tip.y]))

        if thumb_index_distance < 0.05:  # Small distance implies closed hand
            claw_angle = claw_close
        else:
            claw_angle = claw_open

        # Send servo angles to Arduino
        if not debug:
            ser.write(f"B{int(base_angle)}R{int(right_servo_angle)}L{int(left_servo_angle)}E{int(claw_angle)}\n".encode())

        # Draw landmarks and connections on the image
        mp.solutions.drawing_utils.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    # Display current servo angles on the screen
    cv2.putText(image, f"Base: {int(base_angle)}, Left: {int(left_servo_angle)}, Right: {int(right_servo_angle)}, Claw: {int(claw_angle)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    # Show the image
    cv2.imshow('Robotic Arm Control', image)

    # Break the loop on 'Esc' key press
    if cv2.waitKey(5) & 0xFF == 27:
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
