import cv2
import mediapipe as mp
import serial
import time

# Configuration
debug = False  # Set to False when using with Arduino
cam_source = 0  # 0 for default camera, or use IP camera URL

# Serial configuration
if not debug:
    try:
        ser = serial.Serial('COM4', 115200, timeout=1)
        time.sleep(2)  # Allow time for connection to establish
    except serial.SerialException:
        print("Could not open serial port. Make sure Arduino is connected.")
        ser = None
else:
    ser = None

# Servo angle ranges
x_min, x_mid, x_max = 0, 75, 150
y_min, y_mid, y_max = 0, 90, 180
z_min, z_mid, z_max = 10, 90, 180
claw_open, claw_close = 60, 0

# Hand landmark thresholds
wrist_y_min, wrist_y_max = 0.3, 0.9
palm_size_min, palm_size_max = 0.1, 0.3
palm_angle_min, palm_angle_mid = -50, 20

# Initialize servo angles
servo_angle = [x_mid, y_mid, z_mid, claw_open]
prev_servo_angle = servo_angle.copy()

# Utility functions
clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
map_range = lambda x, in_min, in_max, out_min, out_max: int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# MediaPipe initialization
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

def is_hand_closed(landmarks):
    thumb_tip = landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    distance = ((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)**0.5
    return distance < 0.05

def calculate_servo_angles(hand_landmarks):
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    index_mcp = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
    
    # Calculate palm size (distance between wrist and index MCP)
    palm_size = ((wrist.x - index_mcp.x)**2 + (wrist.y - index_mcp.y)**2)**0.5
    
    # X-axis (base rotation)
    angle_x = (wrist.x - index_mcp.x) / palm_size
    angle_x = int(angle_x * 180 / 3.14159)
    angle_x = clamp(angle_x, palm_angle_min, palm_angle_mid)
    servo_x = map_range(angle_x, palm_angle_min, palm_angle_mid, x_max, x_min)
    
    # Y-axis (up/down)
    wrist_y = clamp(wrist.y, wrist_y_min, wrist_y_max)
    servo_y = map_range(wrist_y, wrist_y_min, wrist_y_max, y_max, y_min)
    
    # Z-axis (forward/backward)
    palm_size = clamp(palm_size, palm_size_min, palm_size_max)
    servo_z = map_range(palm_size, palm_size_min, palm_size_max, z_max, z_min)
    
    # Claw
    servo_claw = claw_close if is_hand_closed(hand_landmarks) else claw_open
    
    return [servo_x, servo_y, servo_z, servo_claw]

# Camera setup
cap = cv2.VideoCapture(cam_source)

with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
            
            # Use the first detected hand for control
            servo_angle = calculate_servo_angles(results.multi_hand_landmarks[0])
            
            if servo_angle != prev_servo_angle:
                print("Servo angles:", servo_angle)
                prev_servo_angle = servo_angle.copy()
                if ser:
                    try:
                        # Send data in the format: B{base}L{left}R{right}E{claw}
                        command = f"B{servo_angle[0]}L{servo_angle[1]}R{servo_angle[2]}E{servo_angle[3]}\n"
                        ser.write(command.encode())
                        ser.flush()
                    except serial.SerialException as e:
                        print(f"Error sending data to Arduino: {e}")

        # Flip the image horizontally for a selfie-view display.
        image = cv2.flip(image, 1)
        cv2.putText(image, f"Angles: {servo_angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Hand-Controlled Robotic Arm', image)

        if cv2.waitKey(5) & 0xFF == 27:  # Press 'Esc' to exit
            break

cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()