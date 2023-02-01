#!/usr/bin/env python3
"""Interactive joystick axis and axis polarity remapping demo."""

from tigerasi.tiger_controller import TigerController
from tigerasi.device_codes import JoystickInput, JoystickPolarity

# Uncomment for some prolific log statements.
# import logging
# logger = logging.getLogger()
# logger.setLevel(logging.DEBUG)
# logger.addHandler(logging.StreamHandler())
# logger.handlers[-1].setFormatter(
#    logging.Formatter(fmt='%(asctime)s:%(levelname)s: %(message)s'))

PORT_NAME = "COM10"  # a string indicating the port name.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")
print("Joystick axis mapping:")
axis_mapping = box.get_joystick_axis_mapping('x')
box.set_joystick_axis_polarity(x=JoystickPolarity.DEFAULT)
input("X axis polarity is set to default. Use the joystick to move the x axis."
      "Press Enter when done.")
box.set_joystick_axis_polarity(x=JoystickPolarity.INVERTED)
input("X axis polarity is inverted. Use the joystick to move the x axis. "
      "The direction should be inverted. Press Enter when done.")
box.set_joystick_axis_polarity(x=JoystickPolarity.DEFAULT)
print("X axis polarity is reset to default.")
print("Binding Joystick Y to Tiger X axis stage.")
box.bind_axis_to_joystick_input(x=JoystickInput.JOYSTICK_Y)
input("X axis movement is now tied to up-down movement of the y axis. "
      "Press Enter when done.")
print("Restoring axis mapping.")
box.bind_axis_to_joystick_input(**axis_mapping)
