#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController
import pprint
import time


PORT_NAME = "COM3"# a string indicating the port name.
# port name can be left as None on Linux if udev rules were installed.

print("Connecting to Tiger Controller... ", end=" ", flush=True)
tigerbox = TigerController(PORT_NAME)
print("Done.")

print("Box Configuration:")
pprint.pprint(tigerbox.get_build_config())

print("Test PM control of axis")
tigerbox.pm(v=0)

print("Test scan commands")
tigerbox.scanr(x=0, y=1)
tigerbox.scanv(x=0, y=0, z=1)
tigerbox.scan()

print("Test PZINFO")
print(tigerbox.get_pzinfo(card_address = "35"))

print("End of demo.")
