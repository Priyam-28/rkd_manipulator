import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# Set debug to False to send data to Arduino
debug = False

# Initialize serial communication (adjust port for your system)
if not debug:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) # Update COM port as necessary
    time.sleep(2)  # Wait for the serial connection to initialize

# Servo angle ranges
base_min, base_max = 0, 180  # Base servo range
left_servo_min, left_servo_max = -30, 80  # Left servo range
right_servo_min, right_servo_max = 0, 180  # Right servo range
claw_open, claw_close = 0, 180  # Claw servo open/close range

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Utility functions
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
map_range = lambda x, in_min, in_max, out_min, out_max: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Initialize webcam
cap = cv2.VideoCapture(0)

# Main loop for hand detection and servo control
with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip image for selfie view and convert color
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        # Default servo angles
        base_angle = 90
        left_servo_angle = 45
        right_servo_angle = 90
        claw_angle = claw_open

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Hand left-right movement for base rotation (x-axis)
                wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                base_angle = map_range(wrist.x, 0.0, 1.0, base_min, base_max)
                base_angle = clamp(base_angle, base_min, base_max)

                # Distance between thumb and index finger for right servo
                thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                distance_thumb_index = np.linalg.norm(
                    np.array([thumb_tip.x, thumb_tip.y]) - np.array([index_tip.x, index_tip.y])
                )
                right_servo_angle = map_range(distance_thumb_index, 0.05, 0.3, right_servo_min, right_servo_max)
                right_servo_angle = clamp(right_servo_angle, right_servo_min, right_servo_max)

                # Wrist y-axis movement for left servo
                left_servo_angle = map_range(wrist.y, 0.0, 1.0, left_servo_max, left_servo_min)
                left_servo_angle = clamp(left_servo_angle, left_servo_min, left_servo_max)

                # Open/close hand for claw control (palm open or closed)
                distance_palm = np.linalg.norm(
                    np.array([thumb_tip.x, thumb_tip.y]) - np.array([index_tip.x, index_tip.y])
                )
                if distance_palm < 0.1:
                    claw_angle = claw_close  # Close claw
                else:
                    claw_angle = claw_open  # Open claw

                # Draw hand landmarks
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Display current servo angles on screen
        cv2.putText(image, f"Base: {int(base_angle)}, Left: {int(left_servo_angle)}, Right: {int(right_servo_angle)}, Claw: {int(claw_angle)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Show the image with landmarks
        cv2.imshow('Robotic Arm Control', image)

        # Send servo angles to Arduino
        if not debug:
            ser.write(f"B{int(base_angle)},L{int(left_servo_angle)},R{int(right_servo_angle)},C{int(claw_angle)}\n".encode())

        # Exit on 'Esc' key
        if cv2.waitKey(5) & 0xFF == 27:
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
