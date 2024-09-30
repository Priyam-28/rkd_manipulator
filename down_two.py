import pygame
import serial
import time

# Initialize Pygame for joystick input
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Establish serial communication with Arduino
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Initial end effector position
end_effector_angle = 90

while True:
    pygame.event.pump()

    # Get joystick values (Make sure axis 0 is the base axis)
    base_x = joystick.get_axis(0)  # Left joystick x-axis (base)
    right_y = joystick.get_axis(1)  # Left joystick y-axis (right servo)
    left_y = joystick.get_axis(3)  # Right joystick y-axis (left servo)

    # Map joystick values to servo angles (0-180 for all)
    base_angle = map_value(base_x, -1, 1, 0, 180)
    right_servo_angle = map_value(right_y, -1, 1, 0, 180)
    left_servo_angle = map_value(left_y, -1, 1, 0, 180)

    # Trigger buttons for end effector control
    left_trigger = joystick.get_button(4)  # Left trigger
    right_trigger = joystick.get_button(5)  # Right trigger

    if right_trigger:
        end_effector_angle = 180  # Open end effector
    elif left_trigger:
        end_effector_angle = 0  # Close end effector

    # Throttle sending commands to avoid overloading the serial port
    time.sleep(0.05)  # Reduce send frequency slightly

    # Send data to Arduino
    data_string = f"B{base_angle}R{right_servo_angle}L{left_servo_angle}E{end_effector_angle}\n"
    arduino.write(data_string.encode())

    # Print for debugging
    print(f"Sending to Arduino - Base: {base_angle}, Right Servo: {right_servo_angle}, Left Servo: {left_servo_angle}, End Effector: {end_effector_angle}")
