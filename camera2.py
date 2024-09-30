import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
time.sleep(2)

# Initialize MediaPipe hand tracker
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)  # We need to track only one hand
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
            wrist = hand_landmarks.landmark[0]
            index_finger_tip = hand_landmarks.landmark[8]  # Index finger tip for detailed movement
            thumb_tip = hand_landmarks.landmark[4]

            # Map wrist X-axis to base motor (rotation)
            base_angle = map_range(wrist.x, 0, 1, 0, 180)

            # Map Y-axis of the wrist to control up-down (shoulder equivalent)
            left_servo_angle = map_range(wrist.y, 0, 1, 70, 200)

            # Map index finger tip movement to control the "elbow" (right servo)
            right_servo_angle = map_range(index_finger_tip.y, 0, 1, -90, 210)

            # Send the calculated angles to the Arduino
            arduino.write(f"{base_angle},{left_servo_angle},{right_servo_angle}\n".encode('utf-8'))

            # Print the values for debugging
            print(f"Base Angle: {base_angle}, Left Servo Angle: {left_servo_angle}, Right Servo Angle: {right_servo_angle}")

            # Draw hand landmarks
            mp_draw.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    # Display the camera feed with landmarks
    cv2.imshow("Hand Tracking", img)

    # Break the loop with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
