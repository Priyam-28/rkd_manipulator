import pygame
import serial
import time

# Initialize Pygame for joystick handling
pygame.init()
pygame.joystick.init()

# Set up joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up Arduino connection
arduino = serial.Serial('/dev/ttyACM2', 9600, timeout=1)
time.sleep(2)

# Function to map joystick value (-1 to 1) to servo angle (0 to 180)
def map_joystick_value(value):
    return int((value + 1) * 90)

# Main loop
while True:
    pygame.event.pump()  # Process events to get joystick values
    
    # Get joystick Y-axis values (up-down movement)
    left_y_axis = joystick.get_axis(1)  # Left joystick (Y-axis)
    right_y_axis = joystick.get_axis(4)  # Right joystick (Y-axis)
    
    # Map joystick values to angles (0 to 180 degrees)
    left_servo_angle = map_joystick_value(left_y_axis)
    right_servo_angle = map_joystick_value(right_y_axis)
    
    # Synchronize movement: The servos should remain parallel and aligned
    # Adjust the angles to keep them synchronized
    average_angle = (left_servo_angle + right_servo_angle) // 2
    left_servo_angle = average_angle
    right_servo_angle = average_angle
    
    # Send the angle to Arduino for both servos (servo 1 and servo 2)
    arduino.write(f"{left_servo_angle},{right_servo_angle}\n".encode('utf-8'))
    
    print(f"Left Servo Angle: {left_servo_angle}, Right Servo Angle: {right_servo_angle}")
    
    time.sleep(0.05)  # Small delay to avoid overwhelming the serial communication
