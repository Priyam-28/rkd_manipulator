import pygame
import serial
import time

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Setup serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Adjust '/dev/ttyACM0' to your correct port
time.sleep(2)  # Wait for connection to establish

def send_angles_to_arduino(angle1, angle2):
    # Ensure to send the angles followed by a newline character
    arduino.write(f"{angle1},{angle2}\n".encode())
    print(f"Sent angles: {angle1},{angle2}")

def main():
    previous_angle1 = -1  # Store the previous angle for left joystick
    previous_angle2 = -1  # Store the previous angle for right joystick
    
    while True:
        pygame.event.pump()  # Process pygame events

        # Read the Y-axis of the left joystick (up is negative, down is positive)
        left_y_axis_value = joystick.get_axis(1)
        
        # Read the Y-axis of the right joystick (up is negative, down is positive)
        right_y_axis_value = joystick.get_axis(4)

        # Convert joystick values (-1 to 1) to servo angles (0 to 180)
        angle1 = int((left_y_axis_value + 1) * 90)
        angle2 = int((right_y_axis_value + 1) * 90)

        # Send the angles to Arduino only if they have changed
        if angle1 != previous_angle1 or angle2 != previous_angle2:
            send_angles_to_arduino(angle1, angle2)
            previous_angle1 = angle1
            previous_angle2 = angle2

        time.sleep(0.1)  # Adjust if needed for smoother control

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        arduino.close()
