#!/usr/bin/env python3
"""Demo Joystick disable/enable while restoring axis mapping correctly."""

from tigerasi.tiger_controller import TigerController
import pprint

# Uncomment for some prolific log statements.
# import logging
# logger = logging.getLogger()
# logger.setLevel(logging.DEBUG)
# logger.addHandler(logging.StreamHandler())
# logger.handlers[-1].setFormatter(
#    logging.Formatter(fmt='%(asctime)s:%(levelname)s: %(message)s'))

PORT_NAME = "COM3"  # a string indicating the port name.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")

# Note: re-enabling the joystick will apply a "default" axis mapping,
#   which isn't necessarily correct. To restore state properly, we need to
#   get the current axis mapping, and then reapply it.
axis_mapping = box.get_joystick_axis_mapping()
print("Original axis mapping is: ")
pprint.pprint(axis_mapping)
print("Disabling joystick inputs.")
box.disable_joystick_inputs()
print("Enabling joystick inputs.")
box.enable_joystick_inputs()
print("default restored axis mapping is: ")
pprint.pprint(box.get_joystick_axis_mapping())
box.bind_axis_to_joystick_input(**axis_mapping)
print("Applying saved axis mapping. Restored axis mapping is now:")
pprint.pprint(box.get_joystick_axis_mapping())

