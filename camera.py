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
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Map function
def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Hand gesture detection for end effector (open/close)
def is_claw_open(landmarks):
    # Calculate the distance between thumb tip (landmark 4) and index finger tip (landmark 8)
    thumb_tip = landmarks[4]
    index_tip = landmarks[8]
    
    # Euclidean distance between thumb tip and index tip
    distance = math.sqrt((thumb_tip.x - index_tip.x) ** 2 + (thumb_tip.y - index_tip.y) ** 2)
    
    # Threshold for detecting open hand (distance is relatively larger)
    return distance > 0.1

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

                # Right hand wrist y-axis controls right servo (indexing landmark 0 for wrist)
                if hand_landmarks.landmark[0].y > 0.5:  # Right hand condition
                    right_wrist_y = hand_landmarks.landmark[0].y
                    right_servo_angle = map_value(right_wrist_y, 0, 1, 0, 180)

                    # Claw open/close (right hand)
                    if is_claw_open(hand_landmarks.landmark):
                        end_effector_angle = 180  # Open
                    else:
                        end_effector_angle = 0  # Closed

                # Left hand wrist y-axis controls left servo
                if hand_landmarks.landmark[0].y < 0.5:  # Left hand condition
                    left_wrist_y = hand_landmarks.landmark[0].y
                    left_servo_angle = map_value(left_wrist_y, 0, 1, -30, 100)

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
