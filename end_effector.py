import pygame
import serial
import time

# Initialize Pygame for joystick input
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Establish serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1) 
time.sleep(2) # Change 'COM3' to the correct port

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Initial end effector position
end_effector_angle = 180

while True:
    pygame.event.pump()
    
    # Get joystick values
    base_x = joystick.get_axis(0)  # Left joystick x-axis (base)
    right_y = joystick.get_axis(1)  # Left joystick y-axis (right servo)
    left_y = joystick.get_axis(3)  # Right joystick y-axis (left servo)

    # Map joystick values to servo angles (0-180 for all)
    base_angle = map_value(base_x, -1, 1, 0, 180)
    right_servo_angle = map_value(right_y, -1, 1, 0, 180)
    left_servo_angle = map_value(left_y, -1, 1, -30, 80)

    # Trigger buttons for end effector control
    left_trigger = joystick.get_button(4)  # Left trigger (button index might vary)
    right_trigger = joystick.get_button(5)  # Right trigger (button index might vary)

    if right_trigger:
        end_effector_angle = 180  # Open end effector
    elif left_trigger:
        end_effector_angle = 0  # Close end effector

    # Send data to Arduino
    arduino.write(f"B{base_angle}R{right_servo_angle}L{left_servo_angle}E{end_effector_angle}\n".encode())

    # Print for debugging
    print(f"Base: {base_angle}, Right Servo: {right_servo_angle}, Left Servo: {left_servo_angle}, End Effector: {end_effector_angle}")

    time.sleep(0.1)
