#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController
from tigerasi.device_codes import ScanPattern
import pprint
from time import perf_counter, sleep

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
tigerbox = TigerController(PORT_NAME)
print("Done.")

#print("Box Configuration:")
#pprint.pprint(tigerbox.get_build_config())

print("Test scan commands")
# Conduct one scan line of 100 tiles with the fast_axis in the x direction.
start_x, start_y = (0, 0)
tile_count = 100
tigerbox.set_speed(x=0.01)  # [mm/sec]
tigerbox.setup_scan(fast_axis='x', slow_axis='y', pattern=ScanPattern.RASTER)
tigerbox.scanr(scan_start_mm=start_x, pulse_interval_um=10,
               num_pixels=tile_count)
tigerbox.scanv(scan_start_mm=start_y, scan_stop_mm=start_y, line_count=1)
tigerbox.start_scan()

# Periodically print position.
elapsed_time = 0
start_time = perf_counter()
while elapsed_time < 10:
    elapsed_time = perf_counter() - start_time
    position = tigerbox.get_position('x')
    print(position)
    sleep(0.1)


print("End of demo.")
