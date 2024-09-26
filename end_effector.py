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

# Gripper servo angle range
GRIPPER_OPEN_ANGLE = 90    # Open position (adjust as needed)
GRIPPER_CLOSE_ANGLE = -75  # Close position (adjust as needed)

# Function to map trigger input to gripper angle
def get_gripper_angle(left_trigger, right_trigger):
    if left_trigger > 0.1:  # LT is pressed, open gripper
        return GRIPPER_OPEN_ANGLE
    elif right_trigger > 0.1:  # RT is pressed, close gripper
        return GRIPPER_CLOSE_ANGLE
    else:
        return None  # No change in gripper

# Main loop
while True:
    pygame.event.pump()  # Process events to get joystick values
    
    # Get trigger values (LT and RT)
    left_trigger = joystick.get_axis(2)  # Left trigger (LT)
    right_trigger = joystick.get_axis(5)  # Right trigger (RT)
    
    # Map trigger values to gripper angle
    gripper_angle = get_gripper_angle(left_trigger, right_trigger)
    
    if gripper_angle is not None:
        # Send the gripper angle to Arduino
        arduino.write(f"G,{gripper_angle}\n".encode('utf-8'))
        print(f"Gripper Angle: {gripper_angle}")
    
    time.sleep(0.05)  # Small delay to avoid overwhelming the serial communication
