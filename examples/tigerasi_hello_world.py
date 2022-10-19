#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController
from tigerasi.device_codes import ScanPattern
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
# Conduct one scan line of 100 tiles with the fast_axis in the z direction.
start_z, start_y = (0,0)
tile_count = 100
tigerbox.scanr(scan_start_mm=start_z, pulse_interval_enc_ticks=32,
               num_pixels=tile_count)
tigerbox.scanv(scan_start_mm=start_y, scan_stop_mm=start_y, line_count=1)
z_axis_id = tigerbox.get_axis_id("z")
y_axis_id = tigerbox.get_axis_id("y")
tigerbox.scan(fast_axis_id=z_axis_id, slow_axis_id=y_axis_id,
              pattern=ScanPattern.RASTER)
tigerbox.start_scan()

print("Test PZINFO")
print(tigerbox.get_pzinfo(card_address = "35"))

print("End of demo.")
