#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController
from time import sleep

PORT_NAME = "COM3"  # a string indicating the port name.
# port name can be left as None on Linux if udev rules were installed.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("done.")

# TEST changing speeds.
motor_axes = box.get_build_config()['Motor Axes']
motor_axes = [a.lower() for a in motor_axes if a.isalpha()]  # make all lowercase.
print(f"axes are: {motor_axes}")
old_speeds = {}
new_speeds = {'x': 0.49999999, 'y': 0.251111111, 'z': 0.752222222}
for ax in [a for a in motor_axes if a in new_speeds]:
    speed = box.get_speed(ax)
    old_speeds[ax] = speed
    print(f"{ax} axis speed is {speed}[mm/sec]")
print()

# Change the speeds of some axes.
box.set_speed(**new_speeds)
# print the speed of everything.
for ax in [a for a in motor_axes if a.isalpha()]:
    speed = box.get_speed(ax)
    print(f"{ax} new axis speed is {speed}[mm/sec]")
print()

# Restore old speeds:
box.set_speed(**old_speeds)
# print the speed of everything
for ax in [a for a in motor_axes if a.isalpha()]:
    speed = box.get_speed(ax)
    print(f"{ax} restored axis speed is {speed}[mm/sec]")

# TEST Moving and halting
# read the position.
box.zero_in_place()
print(f"box position is: {box.get_position('x', 'y', 'z')}")
# Send a relative move cmd.
print("moving an axis.")
box.move_relative(x=2000)
sleep(0.05)
# halt immediately after.
print("halting.")
box.halt()
# read the position.
print(f"box position is: {box.get_position('x', 'y', 'z')}")
box.move_absolute(x=0)
