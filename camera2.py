import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

# Initialize MediaPipe hand tracker
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2)  # We need to track both hands
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

    left_hand_x = None
    right_hand_x = None
    right_hand_y = None

    if results.multi_hand_landmarks:
        for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
            # Determine which hand it is
            handedness = hand_info.classification[0].label  # "Left" or "Right"
            
            if handedness == "Left":
                # Left hand (base motor control)
                left_wrist = hand_landmarks.landmark[0]  # Wrist position of left hand
                left_hand_x = left_wrist.x  # X-axis movement controls the base motor
            elif handedness == "Right":
                # Right hand (servo control)
                right_wrist = hand_landmarks.landmark[0]  # Wrist position of right hand
                right_hand_x = right_wrist.x  # X-axis movement controls left servo
                right_hand_y = right_wrist.y  # Y-axis movement controls right servo

            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Map hand positions to servo angles and send to Arduino
    if left_hand_x is not None:
        base_angle = map_range(left_hand_x, 0, 1, 0, 180)  # Map left hand X to base motor angle (0° to 180°)
    else:
        base_angle = 90  # Default position if hand is not detected

    if right_hand_x is not None:
        left_servo_angle = map_range(right_hand_x, 0, 1, -90, 210)  # Map right hand X to left servo (-90° to 210°)
    else:
        left_servo_angle = 60  # Default position for left servo

    if right_hand_y is not None:
        right_servo_angle = map_range(right_hand_y, 0, 1, 70, 200)  # Map right hand Y to right servo (70° to 200°)
    else:
        right_servo_angle = 135  # Default position for right servo

    # Send the calculated angles to the Arduino
    arduino.write(f"{base_angle},{left_servo_angle},{right_servo_angle}\n".encode('utf-8'))

    # Print the values for debugging
    print(f"Base Angle: {base_angle}, Left Servo Angle: {left_servo_angle}, Right Servo Angle: {right_servo_angle}")

    # Display the camera feed with landmarks
    cv2.imshow("Hand Tracking", img)

    # Break the loop with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
