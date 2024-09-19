import pygame
import time

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick name: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")

while True:
    pygame.event.pump()
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i):
            print(f"Button {i} pressed")
    time.sleep(0.1)
