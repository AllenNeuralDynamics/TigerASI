#!/usr/bin/env python3
"""Connects to the Tiger Box, moves some axes, returns to starting pose."""

from tigerasi.tiger_controller import TigerController
from tigerasi.device_codes import TTLIn0Mode, TTLOut0Mode, ScanPattern, \
    RingBufferMode
import pprint
from time import perf_counter, sleep
import json

PORT_NAME = "COM3"
print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")
build_config = box.get_build_config()
ordered_axes = build_config['Motor Axes']
settings = {}
for axis in ordered_axes:
	settings[f'{axis} Axis']=box.get_info(axis)
with open("tiger_settings.json", "w") as outfile:
    json.dump(settings, outfile, indent=4)
