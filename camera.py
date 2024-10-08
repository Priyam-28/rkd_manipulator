import cv2
import mediapipe as mp
import serial
import time
import math

# Initialize serial communication
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust the port name as needed
time.sleep(2)  # Wait for the Arduino to initialize

# Initialize MediaPipe for hand tracking
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Map function
def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Calculate Euclidean distance between two points
def calculate_distance(landmark1, landmark2):
    return math.sqrt((landmark1.x - landmark2.x) ** 2 + (landmark1.y - landmark2.y) ** 2)

# Check if the hand is open (palm open)
def is_palm_open(landmarks):
    # Calculate distance between wrist and middle finger tip (landmark 9 and 0 for wrist)
    wrist = landmarks[0]
    middle_tip = landmarks[12]
    distance = calculate_distance(wrist, middle_tip)
    return distance > 0.3  # Threshold for detecting an open palm

# Check finger curl (compare tip and knuckle positions)
def is_finger_curled(tip, pip):
    distance = calculate_distance(tip, pip)
    return distance < 0.05  # Threshold for detecting curl

# Hand gesture detection for left servo (thumb curl)
def is_thumb_curled(landmarks):
    thumb_tip = landmarks[4]  # Thumb tip
    thumb_pip = landmarks[2]  # Thumb pip (proximal interphalangeal joint)
    return is_finger_curled(thumb_tip, thumb_pip)  # True if thumb is curled

# Main function
def main():
    cap = cv2.VideoCapture(0)
    base_angle = 90
    right_servo_angle = 90
    left_servo_angle = 35  # Left servo mapped from -30 to 100
    end_effector_angle = 90

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Flip the frame for a mirror-like display and process it
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(frame_rgb)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Draw the hand landmarks on the screen
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Base movement (x-axis hand movement controls base)
                hand_x = hand_landmarks.landmark[0].x  # Wrist landmark
                base_angle = map_value(hand_x, 0, 1, 0, 180)

                # Right servo controlled by y-axis hand movement
                hand_y = hand_landmarks.landmark[0].y  # Wrist landmark
                right_servo_angle = map_value(hand_y, 0, 1, 0, 180)

                # Left servo controlled by thumb curl
                if is_thumb_curled(hand_landmarks.landmark):
                    left_servo_angle = -30  # Full curl
                else:
                    left_servo_angle = 80  # Fully extended

                # Claw open/close (controlled by palm open/close)
                if is_palm_open(hand_landmarks.landmark):
                    end_effector_angle = 180  # Open
                else:
                    end_effector_angle = 0  # Closed

                # Send data to Arduino
                data_string = f"B{base_angle}R{right_servo_angle}L{left_servo_angle}E{end_effector_angle}\n"
                arduino.write(data_string.encode())

                # Display data on the terminal for debugging    
                print(f"Base: {base_angle}, Right Servo: {right_servo_angle}, Left Servo: {left_servo_angle}, End Effector: {end_effector_angle}")

        # Display the frame
        cv2.imshow('Hand Gesture Control', frame)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
