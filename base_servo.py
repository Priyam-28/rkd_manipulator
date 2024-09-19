import pygame
import serial
import time

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Connect to the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Setup serial communication with the Arduino
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)  # Adjust port if needed
time.sleep(2)  # Allow time for Arduino to initialize

def send_angle_to_arduino(angle):
    arduino.write(f"{angle}\n".encode())  # Send angle to Arduino
    print(f"Sent angle: {angle}")         # Print sent angle for debugging

def main():
    while True:
        pygame.event.pump()  # Process pygame events
        
        # Read the X-axis (left-right movement) of the left joystick (axis 0)
        x_axis_value = joystick.get_axis(0)
        
        # Map joystick value (-1 to 1) to servo angle (0 to 180)
        angle = int((x_axis_value + 1) * 90)  # Shift the range [-1, 1] to [0, 180]
        
        # Send the angle to Arduino
        send_angle_to_arduino(angle)
        
        time.sleep(0.1)  # Small delay to prevent flooding the serial port

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        arduino.close()  # Close the serial connection
