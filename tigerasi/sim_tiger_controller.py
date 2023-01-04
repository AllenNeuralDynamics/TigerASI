#!/usr/bin/env python3
"""TigerController Simulated Abstraction"""
from .device_codes import *
from .tiger_controller import axis_check

# TODO: consider mocking the serial port directly OR
#   consider mocking the replies.

DEFAULT_ENC_TICKS_TO_MM = 181590.4


class TigerController:
    """Tiger Box Serial Port Abstraction."""

    # Constants
    BAUD_RATE = 115200
    TIMEOUT = 5

    def __init__(self, com_port,
                 build_config={'Motor Axes': ['X', 'Y', 'Z', 'M', 'N']}):
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
    @axis_check('wait_for_reply', 'wait_for_output')
    def move_axes_relative(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """move the axes specified in kwargs by a relative amount.

        Note: Units are in tenths of microns."""
        axes_str = ""
        for key, val in kwargs.items():
            self.sim_positions[key.lower()] += val
        # TODO; add some sleeping here.

    @axis_check('wait_for_reply', 'wait_for_output')
    def move_axes_absolute(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """move the axes specified in kwargs by the specified absolute amount (in tenths of microns)."""
        axes_str = ""
        for key, val in kwargs.items():
            self.sim_positions[key.lower()] = val
        # TODO; add some sleeping here.

    @axis_check('wait_for_reply', 'wait_for_output')
    def zero_in_place(self, *args: str):
        """Zero out the specified axes"""
        axis_positions = {}
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in args:
            axis_positions[axis] = 0
        self.set_position(**axis_positions)

    @axis_check('wait_for_reply', 'wait_for_output')
    def set_position(self, **kwargs: float):
        """Set the specified axes to the specified positions."""
        axes_str = ""
        for axis, val in kwargs.items():
            self.sim_positions[axis.lower] = val
        # TODO; add some sleeping here.
        
    def set_axis_backlash(self, **kwargs: float):
        pass

    @axis_check('wait_for_reply', 'wait_for_output')
    def get_position(self, *args: str):
        """return the controller's locations.

        returns: a dict keyed by uppercase lettered axis who's value is
                 the position (float).
        """
        # Fill out all args if none are populated.
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        axes_positions = [self.sim_positions[ax.lower()] for ax  in args]
        return {k: v for k, v in zip(args, axes_positions)}

    def get_encoder_ticks_per_mm(self, axis: str):
        return DEFAULT_ENC_TICKS_TO_MM

    def is_moving(self):
        """blocks. True if any axes is moving. False otherwise."""
        return False

    # Low-Level Commands.
    def send(self, cmd_str : str, read_until: str = "\r\n",
             wait_for_output=True, wait_for_reply=True):
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

    @axis_check('mode')
    def setup_ring_buffer(self, *axes: str,
                          mode: RingBufferMode = RingBufferMode.TTL,
                          wait_for_reply: bool = True,
                          wait_for_output: bool = False):
        pass

    def reset_ring_buffer(self):
        pass

    @axis_check('wait_for_reply', 'wait_for_output')
    def queue_buffered_move(self, wait_for_reply: bool = True,
                            wait_for_output: bool = True, **axes: float,):
        pass

    def set_ttl_pin_modes(self, in0_mode: TTLIn0Mode = None,
                          out0_mode: TTLOut0Mode = None,
                          reverse_output_polarity: bool = False,
                          aux_io_state: int = None,
                          aux_io_mask: int = None,
                          aux_io_mode: int = None,
                          card_address: int = None,
                          wait_for_reply: bool = True,
                          wait_for_output: bool = True):
        pass

    def clear_incoming_message_queue(self):
        """Clear input buffer and reset skipped replies."""
        self.skipped_replies = 0


if __name__ == '__main__':
    import pprint
    import time

    box = TigerController("COM10")
