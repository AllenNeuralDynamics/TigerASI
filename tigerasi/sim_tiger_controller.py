#!/usr/bin/env python3
"""TigerController Simulated Abstraction"""
from serial import SerialException
from .device_codes import *
from .tiger_controller import axis_check
from enum import Enum

# TODO: consider mocking the serial port directly OR
#   consider mocking the replies.


class TigerController:
    """Tiger Box Serial Port Abstraction."""

    # Constants
    BAUD_RATE = 115200
    TIMEOUT = 5

    def __init__(self, com_port, build_config={'Motor Axes': ['X', 'Y', 'Z', 'M', 'N']}):
        self.ser = None
        self.skipped_replies = 0
        self.ser = None

        # Get the lettered axes: ['X', 'Y', 'Z', ...].
        self.build_config = build_config
        self.ordered_axes = build_config['Motor Axes']
        # Create O(1) lookup container.
        self.axes = set(self.ordered_axes)
        self.sim_positions = {ax.lower(): 0 for ax in self.axes}

    # High-Level Commands
    @axis_check
    def move_axes_relative(self, **kwargs: int):
        """move the axes specified in kwargs by a relative amount.

        Note: Units are in tenths of microns."""
        axes_str = ""
        for key, val in kwargs.items():
            self.sim_positions[key.lower()] += val
        # TODO; add some sleeping here.

    @axis_check
    def move_axes_absolute(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """move the axes specified in kwargs by the specified absolute amount (in tenths of microns)."""
        axes_str = ""
        for key, val in kwargs.items():
            self.sim_positions[key.lower()] = val
        # TODO; add some sleeping here.

    @axis_check
    def home_in_place(self, *args: str):
        """Zero out the specified axes"""
        axis_positions = {}
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in args:
            axis_positions[axis] = 0
        self.set_position(**axis_positions)

    @axis_check
    def set_position(self, **kwargs: float):
        """Set the specified axes to the specified positions."""
        axes_str = ""
        for axis, val in kwargs.items():
            self.sim_positions[axis.lower] = val
        # TODO; add some sleeping here.
        
    def set_axis_backlash(self, **kwargs: float):
        pass

    @axis_check
    def get_position(self, *args: str):
        """return the controller's locations.

        returns: a dict keyed by uppercase lettered axis who's value is
                 the position (float).
        """
        # Fill out all args if none are populated.
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        axes_positions = [self.sim_position[ax.lower()] for ax  in args]
        return {k: v for k, v in zip(args, axes_positions)}

    def is_moving(self):
        """blocks. True if any axes is moving. False otherwise."""
        return False

    # Low-Level Commands.
    def send(self, cmd_bytestr : bytes, wait_for_output=True, wait_for_reply=True):
        """Send a command; optionally wait for various conditions.

        param wait: wait until the serial port finishes sending the message.
        param wait_for_output: wait until all outgoing bytes exit the PC.
        param wait_for_reply: wait until at least one line has been read in
                              by the PC.
        """
        pass

    def get_build_config(self):
        """return the configuration of the Tiger Controller.

        returns: a dict that looks like:
            {'Axis Addr': [],
             'Axis Props': ['74', '10', '2', etc.], # these are positions
             'Axis Types': ['x', 'x', 'z', etc],
             'Hex Addr': [],
             'Motor Axes': ['X', 'Y', 'Z', etc]}
        """
        return self.build_config

    def clear_incoming_message_queue(self):
        """Clear input buffer and reset skipped replies."""
        self.skipped_replies = 0


if __name__ == '__main__':
    import pprint
    import time

    box = TigerController("COM10")
