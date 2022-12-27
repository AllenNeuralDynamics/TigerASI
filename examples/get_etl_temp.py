#!/usr/bin/env python3
"""Connects to the Tiger Box, checks temperature of ETL on V axis."""

from tigerasi.tiger_controller import TigerController

PORT_NAME = "COM3"
print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")
temperature = box.get_etl_temp('V')
print(f"ETL temperature is currently {temperature} deg. C")