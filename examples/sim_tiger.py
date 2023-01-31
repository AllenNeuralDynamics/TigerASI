#!/usr/bin/env python3
"""Tiger example; test command formatting without any hardware."""

from tigerasi.sim_tiger_controller import SimTigerController
import logging

# Print log messages to the screen so we can see every outgoing tiger message.
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())
logger.handlers[-1].setFormatter(
   logging.Formatter(fmt='%(asctime)s:%(levelname)s: %(message)s'))

box = SimTigerController()

print("Basic XYZ movement testing.")
box.home('x', 'y', 'z')
print(box.get_position('x', 'y', 'z'))
print("Relative move.")
box.move_relative(x=50)
print(box.get_position('x'))
box.set_position(x=10)
print(box.get_position('x'))
print("Absolute move.")
box.move_absolute(x=50, y=100, z=150)
print()

print("Setting Travel limits.")
box.set_lower_travel_limit(x=-100.0, y=-150.0)
print()

print("Setting axis backlash.")
box.set_axis_backlash(x=40, z=0)
print()

print("Getting speeds.")
speeds = box.get_speed('x', 'y')
print()

print("Setting speeds.")
box.set_speed(**speeds)
print()
