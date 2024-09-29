import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

# Initialize MediaPipe hand tracker
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# Function to map hand landmark position to servo angle
def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Initialize the camera
cap = cv2.VideoCapture(0)

# Main loop
while True:
    success, img = cap.read()
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Get landmark positions for controlling servos
            wrist = hand_landmarks.landmark[0]  # Wrist position
            index_tip = hand_landmarks.landmark[8]  # Index finger tip
            thumb_tip = hand_landmarks.landmark[4]  # Thumb tip
            pinky_tip = hand_landmarks.landmark[20]  # Pinky tip

            # Calculate relative positions
            wrist_x = wrist.x
            wrist_y = wrist.y
            index_y = index_tip.y
            thumb_x = thumb_tip.x
            pinky_x = pinky_tip.x

            # Map movements to servo angles
            base_angle = map_range(thumb_x, 0, 1, 0, 180)  # Thumb x controls the base motor (0° to 180°)
            right_angle = map_range(index_y, 0, 1, 70, 200)  # Index y controls right servo (70° to 200°)
            left_angle = map_range(wrist_y, 0, 1, -90, 210)  # Wrist y controls left servo (-90° to 210°)

            # Send servo angles to Arduino
            arduino.write(f"{base_angle},{right_angle},{left_angle}\n".encode('utf-8'))

            # Display angles for debugging
            print(f"Base Angle: {base_angle}, Right Servo Angle: {right_angle}, Left Servo Angle: {left_angle}")

    # Display the camera feed with landmarks
    cv2.imshow("Hand Tracking", img)

    # Break loop with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
