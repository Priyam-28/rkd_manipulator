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

def send_angle_to_arduino(angle):
    # Ensure to send the angle followed by a newline character
    arduino.write(f"{angle}\n".encode())
    print(f"Sent angle: {angle}")

def main():
    previous_angle = -1  # Store the previous angle to avoid redundant transmissions
    while True:
        pygame.event.pump()  # Process pygame events

        # Read the X-axis of the left joystick (0 is left, 1 is right)
        x_axis_value = joystick.get_axis(0)

        # Convert joystick value (-1 to 1) to servo angle (0 to 180)
        angle = int((x_axis_value + 1) * 90)

        # Send the angle to Arduino only if it has changed
        if angle != previous_angle:
            send_angle_to_arduino(angle)
            previous_angle = angle

        time.sleep(0.1)  # Adjust if needed for smoother control

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        arduino.close()
