#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController, UM_TO_STEPS
from tigerasi.device_codes import ScanPattern
import pprint
import time
import pprint

# Uncomment for some prolific log statements.
import logging
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())
logger.handlers[-1].setFormatter(
   logging.Formatter(fmt='%(asctime)s:%(levelname)s: %(message)s'))

PORT_NAME = "COM3"  # a string indicating the port name.
# port name can be left as None on Linux if udev rules were installed.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")

axes = ['x', 'y', 'z']

box.reset_lower_travel_limits(*axes)
box.reset_upper_travel_limits(*axes)

lower_stage_limits = box.get_lower_travel_limit(*axes)
upper_stage_limits = box.get_upper_travel_limit(*axes)
print("Previous lower stage limits:")
pprint.pprint(lower_stage_limits)
print("Previous upper stage limits:")
pprint.pprint(upper_stage_limits)

print("Beginning interactive stage calibration sequence.")
print("Press CTRL-C to bail at any time.")
for x in axes:
    input(f"Use the joystick to move to the LOWER travel limit of {x.upper()}."
          f" Press enter when ready.")
    axis_pos_dict = box.get_position(x)
    axis_pos_dict_mm = {x: v/UM_TO_STEPS/1000.
                        for x, v in axis_pos_dict.items()}
    box.set_lower_travel_limit(**axis_pos_dict_mm)
    input(f"Use the joystick to move to the UPPER travel limit of {x.upper()}."
          f" Press enter when ready.")
    axis_pos_dict = box.get_position(x)
    axis_pos_dict_mm = {x: v / UM_TO_STEPS / 1000.
                        for x, v in axis_pos_dict.items()}
    box.set_upper_travel_limit(**axis_pos_dict_mm)


lower_stage_limits = box.get_lower_travel_limit(*axes)
upper_stage_limits = box.get_upper_travel_limit(*axes)
print("New lower stage limits:")
pprint.pprint(lower_stage_limits)
print("New upper stage limits:")
pprint.pprint(upper_stage_limits)
