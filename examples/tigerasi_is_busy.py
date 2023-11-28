#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController


PORT_NAME = "COM4"  # a string indicating the port name.
# port name can be left as None on Linux if udev rules were installed.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("done.")

# Start moving the x and y axes.
box.move_relative(x=-2000, y=-2000)
axis_moving_states = box.are_axes_moving()
axis_moving_states_subset = box.are_axes_moving('x', 'y')
x_is_moving = box.is_axis_moving('x')
print(f"moving states (all axes): {axis_moving_states}")
print(f"moving states (x and y): {axis_moving_states_subset}")
print(f"x axis moving state: {x_is_moving}")
box.move_relative(x=2000, y=2000)