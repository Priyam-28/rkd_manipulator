import pygame
import serial
import time

# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Initialize Joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up serial connection (use /dev/ttyUSB0 for Linux or COMx for Windows)
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Change this to your serial port

def read_controller():
    pygame.event.pump()
    x_axis = joystick.get_axis(0)  # X axis of the joystick
    return x_axis

while True:
    x = read_controller()
    
    # Print X-axis value
    print("Joystick X-axis value: {}".format(x))

    # Map joystick input to servo value (example mapping [-1, 1] to [0, 180])
    servo_angle = int(x * 90 + 90)  # Map from [-1,1] to [0,180]

    # Debugging: Print mapped servo angle
    print("Mapped Servo Angle: {}".format(servo_angle))

    # Send data to ESP32 in a correct format
    command = "{}\n".format(servo_angle)  # Using .format() for compatibility
    ser.write(command.encode())  # Encode to send through serial
    
    time.sleep(0.1)  # Adjust delay as needed
