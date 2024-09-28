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

# Function to map joystick value (-1 to 1) to servo angle within a specific range
def map_joystick_value(value, min_angle, max_angle):
    return int((value + 1) * (max_angle - min_angle) / 2 + min_angle)

# Deadzone to prevent small joystick movements from affecting the servos
DEADZONE = 0.1

# Main loop
while True:
    pygame.event.pump()  # Process joystick events
    
    # Get joystick values for all three servos
    base_x_axis = joystick.get_axis(0)  # Left joystick X-axis (base motor)
    right_y_axis = joystick.get_axis(1)  # Left joystick Y-axis (right servo)
    left_y_axis = joystick.get_axis(3)  # Right joystick Y-axis (left servo)
    
    # Map the joystick values to the appropriate servo angles
    if abs(base_x_axis) > DEADZONE:
        base_angle = map_joystick_value(base_x_axis, 0, 180)
    else:
        base_angle = None  # No movement

    if abs(right_y_axis) > DEADZONE:
        right_angle = map_joystick_value(right_y_axis, 70, 200)
    else:
        right_angle = None  # No movement

    if abs(left_y_axis) > DEADZONE:
        left_angle = map_joystick_value(left_y_axis, -90, 210)
    else:
        left_angle = None  # No movement

    # Only send updated angles to Arduino if they exist (not None)
    if base_angle is not None and right_angle is not None and left_angle is not None:
        arduino.write(f"{base_angle},{right_angle},{left_angle}\n".encode('utf-8'))
        print(f"Base Angle: {base_angle}, Right Servo Angle: {right_angle}, Left Servo Angle: {left_angle}")
    
    time.sleep(0.1)  # Small delay to avoid overwhelming the serial communication
