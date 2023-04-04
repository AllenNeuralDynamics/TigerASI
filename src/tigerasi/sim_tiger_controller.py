#!/usr/bin/env python3
"""Simulated Tiger Box."""
from tigerasi.device_codes import *
from tigerasi.tiger_controller import axis_check, no_repeated_axis_check
from unittest.mock import NonCallableMock
from tigerasi.tiger_controller import *
import logging

# TODO: consider mocking the serial port directly OR
#   consider mocking the replies.

DEFAULT_ENC_TICKS_TO_MM = 181590.4

# TODO: finish implementing this.
class SerialStub:

    def __init__(self, com_port, baud_rate, timeout):
        self.in_buffer = bytearray()
        self.out_buffer = bytearray()

    def reset_input_buffer(self):
        self.in_buffer = bytearray()

    def reset_output_buffer(self):
        self.out_buffer = bytearray()

    def write(self, msg: bytes):
        pass

    @property
    def out_waiting(self):
        return 0

    def read_until(self, sep: bytes):
        # return anything that has been pushed into the reply buffer or
        # nothing.
        out_buffer_parts = self.out_buffer.partition(sep)
        self.out_buffer = out_buffer_parts[-1]
        return out_buffer_parts[0]

    def push_to_out_buffer(self, msg: bytes):
        self.out_buffer.extend(msg)


class SimTigerController(TigerController):
    """Simulated Tiger Box Hardware.

        This object lightly tracks its state (axis positions) to fake
        interactions with the real device.

        Data can also be pushed into the fake serial port to test the format
        of outgoing messages.
    """

    # Note: since this class inherits from TigerController, only methods
    #   that (1) require state tracking or (2) require non-empty replies need
    #   to be overwritten.

    def __init__(self, com_port = None,
                 build_config={'Motor Axes': ['X', 'Y', 'Z', 'M', 'N']}):
        self.ser = SerialStub(com_port, None, None)
        self.log = logging.getLogger(__name__)
        self.skipped_replies = 0

        # Get the lettered axes: ['X', 'Y', 'Z', ...].
        self.build_config = build_config
        self.ordered_axes = build_config['Motor Axes']
        # Create O(1) lookup container.
        self.axes = set(self.ordered_axes)
        self.sim_positions = {x.upper(): 0 for x in self.axes}
        self.sim_speeds = {x.upper(): DEFAULT_SPEED_MM_PER_SEC
                           for x in self.axes if not x.isnumeric()}

    @axis_check('wait')
    @no_repeated_axis_check
    def home(self, *axes: str, wait: bool = True):
        if not axes:
            # Default to all lettered axes.
            axes = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in axes:
            self.sim_positions[axis] = 0
        self.ser.push_to_out_buffer(b'\r\n')  # Fake reply.
        super().home(*axes)

    # High-Level Commands
    @axis_check('wait')
    def move_relative(self, wait: bool = True, **axes: int):
        """move the axes specified in kwargs by a relative amount.

        Note: Units are in tenths of microns."""
        for key, val in axes.items():  # Update simulated location.
            self.sim_positions[key] += val
        self.ser.push_to_out_buffer(b'\r\n')  # Fake reply.
        super().move_relative(wait=wait, **axes)

    @axis_check('wait')
    def move_absolute(self, wait: bool = True, **axes: int):
        """move the axes specified by the specified absolute amount
        (in tenths of microns)."""
        for key, val in axes.items():
            self.sim_positions[key] = val
        self.ser.push_to_out_buffer(b'\r\n')  # Fake reply.
        super().move_absolute(wait=wait, **axes)

    @axis_check('wait')
    def zero_in_place(self, *args: str):
        """Zero out the specified axes"""
        axis_positions = {}
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in args:
            axis_positions[axis] = 0
        self.set_position(**axis_positions)

    @axis_check('wait')
    def set_position(self, wait: bool = True, **axes: float):
        """Set the specified axes to the specified positions."""
        axes_str = ""
        for axis, val in axes.items():
            self.sim_positions[axis] = val
        self.ser.push_to_out_buffer(b'\r\n')  # Fake reply.
        super().set_position(wait=wait, **axes)

    @axis_check()
    def get_position(self, *args: str):
        """return the controller's simulated locations.

        returns: a dict keyed by uppercase lettered axis who's value is
                 the position (float).
        """
        # Fill out all args if none are populated.
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        axes_positions = [self.sim_positions[ax] for ax  in args]
        return {k: v for k, v in zip(args, axes_positions)}

    def set_speed(self, wait: bool = True, **axes: float):
        self.sim_speeds.update(**axes)
        self.ser.push_to_out_buffer(f"{ACK}\r\n".encode('ascii'))
        super().set_speed(wait=wait, **axes)

    @axis_check()
    def get_speed(self, *axes: str):
        speed_settings = "".join([f" {x}={round(self.sim_speeds[x], MM_SCALE)}"
                                  for x in axes])
        msg = f"{ACK} {speed_settings}\r\n".encode('ascii')
        self.ser.push_to_out_buffer(msg)
        return super().get_speed(*axes)

    def get_encoder_ticks_per_mm(self, axis: str):
        return DEFAULT_ENC_TICKS_TO_MM

    def is_moving(self):
        """blocks. True if any axes is moving. False otherwise."""
        return False

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
                          wait: bool = True):
        pass

    def reset_ring_buffer(self):
        pass

    @axis_check('wait')
    def queue_buffered_move(self, wait: bool = True, **axes: float,):
        pass

    def set_ttl_pin_modes(self, in0_mode: TTLIn0Mode = None,
                          out0_mode: TTLOut0Mode = None,
                          reverse_output_polarity: bool = False,
                          aux_io_state: int = None,
                          aux_io_mask: int = None,
                          aux_io_mode: int = None,
                          card_address: int = None,
                          wait: bool = True):
        pass

    def clear_incoming_message_queue(self):
        """Clear input buffer and reset skipped replies."""
        self.skipped_replies = 0

    @axis_check('wait')
    def get_etl_temp(self, axis: str, wait: bool = True):
        return 23.15
