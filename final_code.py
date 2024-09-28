import pygame
import serial
import time

# Initialize Pygame for joystick handling
pygame.init()
pygame.joystick.init()

# Set up joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up Arduino connection (adjust port based on your system)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Delay to allow serial connection to establish

# Function to map joystick value (-1 to 1) to servo angle (0 to 180)
def map_joystick_value(value):
    return int((value + 1) * 90)  # Adjusting the joystick range to servo angle (0-180)

# Servo angle limits and movement smoothness
ANGLE_INCREMENT = 1  # Controls how smooth the movement is (lower = smoother)
DEADZONE = 0.1  # Threshold to ignore small joystick movements

# Initial positions of the servos (base, left, right)
base_angle = 90
left_servo_angle = 90
right_servo_angle = 90

def update_angle(current_angle, joystick_value):
    """Update the servo angle based on joystick input within 0-180 degree limits."""
    if abs(joystick_value) > DEADZONE:
        target_angle = map_joystick_value(joystick_value)
        if current_angle < target_angle:
            current_angle = min(current_angle + ANGLE_INCREMENT, target_angle)
        elif current_angle > target_angle:
            current_angle = max(current_angle - ANGLE_INCREMENT, target_angle)
    return current_angle

def send_servo_data_to_arduino(base_angle, left_servo_angle, right_servo_angle):
    """Send the updated servo angles to Arduino."""
    data = f"{base_angle},{left_servo_angle},{right_servo_angle}\n"
    arduino.write(data.encode('utf-8'))
    print(f"Base: {base_angle}, Left: {left_servo_angle}, Right: {right_servo_angle}")

# Main loop
while True:
    pygame.event.pump()  # Process events to get joystick values
    
    # Get joystick inputs
    x_axis_value = joystick.get_axis(0)  # Left joystick (X-axis for base rotation)
    left_y_axis = joystick.get_axis(1)  # Left joystick (Y-axis for left servo)
    right_y_axis = joystick.get_axis(3)  # Right joystick (Y-axis for right servo)

    # Update angles based on joystick input
    base_angle = update_angle(base_angle, x_axis_value)
    left_servo_angle = update_angle(left_servo_angle, left_y_axis)
    right_servo_angle = update_angle(right_servo_angle, right_y_axis)

    # Send updated angles to Arduino
    send_servo_data_to_arduino(base_angle, left_servo_angle, right_servo_angle)

    time.sleep(0.05)  # Small delay for smoother control
