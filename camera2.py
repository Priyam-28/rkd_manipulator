import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# Setup serial communication (change COM4 to your Arduino's port)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) 
time.sleep(2)

# Servo angle ranges
base_min, base_max = 0, 180  # Base rotation controlled by thumb-index distance
left_servo_min, left_servo_max = 0, 180  # Left servo controlled by wrist y-axis
right_servo_min, right_servo_max = 0, 180  # Right servo controlled by wrist x-axis
claw_open, claw_close = 0, 180  # Claw open and close positions

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize webcam
cap = cv2.VideoCapture(0)

# Utility functions for mapping and clamping values
def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Main loop
with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip the image for a selfie-view display
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the image for hand landmarks
        results = hands.process(image_rgb)

        # Default servo angles
        base_angle = 90
        left_servo_angle = 90
        right_servo_angle = 90
        claw_angle = claw_open

        # Check if hands are detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Thumb and index finger control for base movement
                thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]

                # Calculate the distance between thumb and index finger
                thumb_index_distance = np.linalg.norm(
                    np.array([thumb_tip.x, thumb_tip.y]) - np.array([index_tip.x, index_tip.y])
                )
                
                # Map thumb-index distance to base rotation angle
                base_angle = map_range(thumb_index_distance, 0.05, 0.3, base_min, base_max)
                base_angle = clamp(base_angle, base_min, base_max)

                # Control left servo with wrist's y-axis movement
                left_servo_angle = map_range(wrist.y, 0.1, 0.9, left_servo_min, left_servo_max)
                left_servo_angle = clamp(left_servo_angle, left_servo_min, left_servo_max)

                # Control right servo with wrist's x-axis movement
                right_servo_angle = map_range(wrist.x, 0.1, 0.9, right_servo_min, right_servo_max)
                right_servo_angle = clamp(right_servo_angle, right_servo_min, right_servo_max)

                # Claw control based on palm openness (thumb-index distance)
                if thumb_index_distance < 0.1:  # Assume a closed palm means closing the claw
                    claw_angle = claw_close
                else:
                    claw_angle = claw_open

                # Draw landmarks on the hand
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Send servo data to Arduino
        ser.write(f"{int(base_angle)},{int(left_servo_angle)},{int(right_servo_angle)},{int(claw_angle)}\n".encode())

        # Display current servo angles on screen
        cv2.putText(image, f"Base: {int(base_angle)}, Left: {int(left_servo_angle)}, Right: {int(right_servo_angle)}, Claw: {int(claw_angle)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Show the output frame
        cv2.imshow('Robotic Arm Control', image)

        # Break the loop when 'Esc' is pressed
        if cv2.waitKey(5) & 0xFF == 27:
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
