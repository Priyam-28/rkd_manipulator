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
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust '/dev/ttyACM0' based on your Arduino port
time.sleep(2)  # Wait for connection to establish

def send_angle_to_arduino(angle):
    arduino.write(f"{angle}\n".encode())
    print(f"Sent angle: {angle}")

def main():
    while True:
        pygame.event.pump()
        axis_value = joystick.get_axis(1)  # Read the Y axis of the joystick
        
        # Convert joystick value (-1 to 1) to servo angle (0 to 180)
        angle = int((axis_value + 1) * 90)
        
        # Send the angle to Arduino
        send_angle_to_arduino(angle)
        
        time.sleep(0.1)  # Adjust if needed for smoother control

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        arduino.close()
