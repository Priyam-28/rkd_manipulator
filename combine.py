import serial
import cv2
import mediapipe as mp
import numpy as np
import pygame
import time

# Configurations
write_video = False  # Set to True to record the video
debug = True  # Set to False to send servo angles to Arduino
cam_source = 0  # Set to 0 for the default webcam, or use a video stream URL
use_joystick = False  # Set to True to use joystick instead of hand gestures

if not debug:
    ser = serial.Serial('COM4', 115200)  # Set to your serial port

# Servo angle ranges
base_min, base_max = 0, 180  # Base rotation controlled by thumb-index movement
left_servo_min, left_servo_max = 70, 200  # Left servo controlled by wrist angle
right_servo_min, right_servo_max = -90, 210  # Right servo controlled by forearm angle
claw_open, claw_close = 0, 180  # Claw servo control for open/close

# Initialize MediaPipe Hands
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# Utility functions for mapping and clamping
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
map_range = lambda x, in_min, in_max, out_min, out_max: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Initialize webcam
cap = cv2.VideoCapture(cam_source)

# Video writer for saving
if write_video:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 60.0, (640, 480))

# Initialize Pygame for joystick control
if use_joystick:
    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    time.sleep(2)

# Main loop for hand detection and servo control
with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip the image for selfie-view and convert color
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Default servo angles
        base_angle = 90
        left_servo_angle = 135
        right_servo_angle = 60
        claw_angle = claw_open

        if use_joystick:
            # Use joystick input
            pygame.event.pump()
            base_x = joystick.get_axis(0)  # Left joystick x-axis (base)
            right_y = joystick.get_axis(1)  # Left joystick y-axis (right servo)
            left_y = joystick.get_axis(3)  # Right joystick y-axis (left servo)

            # Map joystick values to servo angles
            base_angle = map_range(base_x, -1, 1, base_min, base_max)
            right_servo_angle = map_range(right_y, -1, 1, right_servo_min, right_servo_max)
            left_servo_angle = map_range(left_y, -1, 1, left_servo_min, left_servo_max)

            # Trigger buttons for end effector control
            left_trigger = joystick.get_button(4)  # Left trigger
            right_trigger = joystick.get_button(5)  # Right trigger

            if right_trigger:
                claw_angle = claw_open  # Open claw
            elif left_trigger:
                claw_angle = claw_close  # Close claw
        else:
            # Process hand landmarks with MediaPipe
            results = hands.process(image_rgb)
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Thumb and index finger control for base movement
                    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                    distance_thumb_index = np.linalg.norm(
                        np.array([thumb_tip.x, thumb_tip.y]) - np.array([index_tip.x, index_tip.y])
                    )

                    # Map the distance to base servo angle
                    base_angle = map_range(distance_thumb_index, 0.05, 0.3, base_min, base_max)
                    base_angle = clamp(base_angle, base_min, base_max)

                    # Wrist and forearm control
                    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]

                    # Use wrist y-axis for left servo (control left)
                    left_servo_angle = map_range(wrist.y, 0.1, 0.9, left_servo_max, left_servo_min)
                    left_servo_angle = clamp(left_servo_angle, left_servo_min, left_servo_max)

                    # Use the angle between wrist and pinky for right servo (forearm control)
                    right_servo_angle = map_range(pinky_tip.y, 0.1, 0.9, right_servo_max, right_servo_min)
                    right_servo_angle = clamp(right_servo_angle, right_servo_min, right_servo_max)

                    # Claw control based on thumb-index distance
                    if distance_thumb_index < 0.1:
                        claw_angle = claw_close  # Close claw
                    else:
                        claw_angle = claw_open  # Open claw

                    # Draw landmarks and connections on the image
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Display the current servo angles on the screen
        cv2.putText(image, f"Base: {int(base_angle)}, Left: {int(left_servo_angle)}, Right: {int(right_servo_angle)}, Claw: {int(claw_angle)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Show the image
        cv2.imshow('Robotic Arm Control', image)

        # Send servo angles to Arduino (skip if debugging)
        if not debug:
            ser.write(f"{int(base_angle)},{int(left_servo_angle)},{int(right_servo_angle)},{int(claw_angle)}\n".encode())

        # Write to video file if enabled
        if write_video:
            out.write(image)

        # Break the loop on 'Esc' key press
        if cv2.waitKey(5) & 0xFF == 27:
            break

# Release resources
cap.release()
if write_video:
    out.release()
cv2.destroyAllWindows()
