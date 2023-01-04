#!/usr/bin/env python3
"""Connects to the Tiger Box, checks temperature of ETL on V axis."""

from tigerasi.tiger_controller import TigerController

# Uncomment for some prolific log statements.
# import logging
# logger = logging.getLogger()
# logger.setLevel(logging.DEBUG)
# logger.addHandler(logging.StreamHandler())
# logger.handlers[-1].setFormatter(
#    logging.Formatter(fmt='%(asctime)s:%(levelname)s: %(message)s'))

PORT_NAME = "COM3"
print("Connecting to Tiger Controller... ", end=" ", flush=True)
box = TigerController(PORT_NAME)
print("Done.")
temperature = box.get_etl_temp('V')
print(f"ETL temperature is currently {temperature} deg. C")