#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController
import pprint
import time


PORT_NAME = "/dev/ttyACM0"# a string indicating the port name.
# port name can be left as None on Linux if udev rules were installed.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
tigerbox = TigerController(PORT_NAME)
print("Done.")

print("Box Configuration:")
pprint.pprint(tigerbox.get_build_config)

print("Setting current location as zero.")
tigerbox.home_in_place()

tigerbox.move_axes_relative(x=10000, y=10000)

tigerbox.move_axes_absolute(x=0, y=0)


print("End of demo.")
