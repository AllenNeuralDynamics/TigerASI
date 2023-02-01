#!/usr/bin/env python3
"""Attach a log handler to print logs to stdout."""

from tigerasi.tiger_controller import TigerController
import logging

# Send log messages to stdout so we can see every outgoing/incoming tiger msg.
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())
logger.handlers[-1].setFormatter(
   logging.Formatter(fmt='%(asctime)s:%(levelname)s: %(message)s'))

box = TigerController()

print("Homing.")
box.home('x', 'y', 'z')
print("Getting position.")
print(box.get_position('x', 'y', 'z'))

