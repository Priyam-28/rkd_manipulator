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
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

# Function to map joystick value (-1 to 1) to servo angle (0 to 180)
def map_joystick_value(value):
    return int((value + 2) * 90)

# Initial positions of the servos
current_up_down_angle = 90  # Starting in the middle
current_left_right_angle = 90  # Starting in the middle

# Deadzone threshold to prevent small movements from the joystick causing updates
DEADZONE = 0.1

# Main loop
while True:
    pygame.event.pump()  # Process events to get joystick values
    
    # Get joystick Y-axis value (up-down movement from left joystick)
    left_y_axis = joystick.get_axis(1)  # Left joystick (Y-axis for up/down)

    # Get joystick X-axis value (left-right movement from right joystick)
    right_x_axis = joystick.get_axis(3)  # Right joystick (X-axis for left/right)
    
    # Only update the up-down servo if the joystick is moved past the deadzone
    if abs(left_y_axis) > DEADZONE:
        current_up_down_angle = map_joystick_value(left_y_axis)
    
    # Only update the left-right servo if the joystick is moved past the deadzone
    if abs(right_x_axis) > DEADZONE:
        current_left_right_angle = map_joystick_value(right_x_axis)
    
    # Send the angle to Arduino for both movements (servo 1 for up-down and servo 2 for left-right)
    arduino.write(f"{current_up_down_angle},{current_left_right_angle}\n".encode('utf-8'))
    
    print(f"Up-Down Servo Angle: {current_up_down_angle}, Left-Right Servo Angle: {current_left_right_angle}")
    
    time.sleep(0.05)  # Small delay to avoid overwhelming the serial communication
