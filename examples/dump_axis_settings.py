#!/usr/bin/env python3
"""Connects to the Tiger Box, grabs and dumps axis information."""

from tigerasi.tiger_controller import TigerController
import json
import pprint

PORT_NAME = "COM3"
print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")
build_config = box.get_build_config()
ordered_axes = build_config['Motor Axes']
settings = {}
for axis in ordered_axes:
    axis_settings = box.get_info(axis)
    print(f"Axis: {axis}")
    pprint.pprint(axis_settings)
    settings[f'{axis} Axis'] = axis_settings

with open("tiger_settings.json", "w") as outfile:
    json.dump(settings, outfile, indent=4)
