#!/usr/bin/env python3
"""TigerController Serial Port Abstraction"""
from serial import Serial, SerialException
from .device_codes import *
from enum import Enum


def axis_check(func):
    """Ensure that the axis (specified as an arg or kwarg) exists."""

    def inner(self, *args, **kwargs):
        # Check if axes are specified in *args.
        # Otherwise, check axes specified in **kwargs.
        iterable = args if len(args) else kwargs.keys()
        for arg in iterable:
            # skip keyworded flags.
            if arg.startswith("wait_") and arg in kwargs:
                continue
            assert arg.upper() in self.axes, \
                f"Error. Axis '{arg.upper()}' does not exist"
        return func(self, *args, **kwargs)
    return inner


class TigerController:
    """Tiger Box Serial Port Abstraction."""

    # Constants
    BAUD_RATE = 115200
    TIMEOUT = 5

    def __init__(self, com_port):
        self.ser = None
        self.skipped_replies = 0
        try:
            self.ser = Serial(com_port, TigerController.BAUD_RATE,
                              timeout=TigerController.TIMEOUT)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except SerialException as e:
            print("Error: could not open connection to Tiger Controller. "
                  "Is the device plugged in? Is another program using it?")
            raise

        # Get the lettered axes: ['X', 'Y', 'Z', ...].
        self.ordered_axes = self.get_build_config()['Motor Axes']
        ## FW-1000 filter wheels have their own command set but show up in
        # axis list as '0', '1' etc, so we remove them..
        self.ordered_filter_wheels = [fw for fw in self.ordered_axes if fw.isnumeric()]
        self.ordered_axes = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        #print(f"ordered axes are: {self.ordered_axes}")
        # Create O(1) lookup container.
        self.axes = set(self.ordered_axes)

    # High-Level Commands
    @axis_check
    def move_axes_relative(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """move the axes specified in kwargs by a relative amount.

        Note: Units are in tenths of microns."""
        axes_str = ""
        for key, val in kwargs.items():
            axes_str += f" {key.upper()}={val}"
        cmd_str = Cmds.MOVEREL.decode('utf8') + axes_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    @axis_check
    def move_axes_absolute(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """move the axes specified in kwargs by the specified absolute amount (in tenths of microns)."""
        axes_str = ""
        for key, val in kwargs.items():
            axes_str += f" {key.upper()}={val}"
        cmd_str = Cmds.MOVEABS.decode('utf8') + axes_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    @axis_check
    def home_in_place(self, *args: str):
        """Zero out the specified axes"""
        # TODO: what happens if we home a device with CLOCKED POSITIONS?
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
            axes_str += f" {axis.upper()}={val}"
        cmd_str = Cmds.HERE.decode('utf8') + axes_str + '\r'
        self.send(cmd_str.encode('ascii'))

    @axis_check
    def get_position(self, *args: str):
        """return the controller's locations for non-numeric axes.

        returns: a dict keyed by uppercase lettered axis who's value is
                 the position (float).

        Note: filter wheels positions are not accessible this way.
        """
        axes_str = ""
        # Fill out all args if none are populated.
        if not args:
            # Default to all lettered axes.
            # Note: numeric (filter wheel) axes would be ignored if we added them.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in args:
            axes_str += f" {axis.upper()}"
        cmd_str = Cmds.WHERE.decode('utf8') + axes_str + '\r'
        reply = self.send(cmd_str.encode('ascii'))
        axes_positions = [float(v) for v in reply.split()[1:]]
        return {k: v for k, v in zip(args, axes_positions)}

    def is_moving(self):
        """blocks. True if any axes is moving. False otherwise."""
        # Send the inquiry.
        reply = self.send(Cmds.STATUS).rstrip('\r\n')
        # interpret reply.
        if reply == "B":
            return True
        elif reply == "N":
            return False
        else:
            raise RuntimeError(f"Error. Cannot tell if device is moving. Received: '{reply}'")

    def clear_incoming_message_queue(self):
        """Clear input buffer and reset skipped replies."""
        self.skipped_replies = 0
        self.ser.reset_input_buffer()

    # Low-Level Commands.
    def send(self, cmd_bytestr : bytes, wait_for_output=True, wait_for_reply=True):
        """Send a command; optionally wait for various conditions.

        param wait: wait until the serial port finishes sending the message.
        param wait_for_output: wait until all outgoing bytes exit the PC.
        param wait_for_reply: wait until at least one line has been read in
                              by the PC.
        """
        #print(f"sending: {repr(cmd_bytestr.decode('utf8'))}")  # for debugging
        self.ser.write(cmd_bytestr)
        if wait_for_output:  # Wait for all bytes to exit the output buffer.
            while self.ser.out_waiting:
                pass
        # If we do not wait for a reply, we must track how many replies to read later.
        if not wait_for_reply :
            self.skipped_replies += 1
            return
        # Every command issues a reply from the TigerController.
        # We must iterate through all skipped replies till we get ours.
        # TigerController delimits multiple lines with '\r' and replies with '\r\n'.
        # We must get all the skipped replies before we can get ours.
        # FIXME: it is possible to overflow the buffer if we don't read enough.
        # Note: reading at least one reply out of the buffer costs ~0.01[s]
        while True:
            reply = self.ser.read_until(b'\r\n').decode("utf8")
            #print(f"reply: {repr(reply)}")  # for debugging
            try:
                self.check_reply_for_errors(reply)
            except SyntaxError as e:  # Technically, this could be a skipped reply.
                print(f"Error occurred when sending: {repr(cmd_bytestr)}")
                raise
            if self.skipped_replies:
                self.skipped_replies -= 1
            else:
                break
        return reply

    def get_build_config(self):
        """return the configuration of the Tiger Controller.

        returns: a dict that looks like:
            {'Axis Addr': [],
             'Axis Props': ['74', '10', '2', etc.], # these are positions
             'Axis Types': ['x', 'x', 'z', etc],
             'Hex Addr': [],
             'Motor Axes': ['X', 'Y', 'Z', etc]}
        """
        reply = self.send(Cmds.BUILD_X)
        # Reply is formatted in such a way that it can be put into dict form.
        return self._reply_to_dict(reply)

    @staticmethod
    def check_reply_for_errors(reply: str):
        """Check if reply contains an error code; returns None or throws exception."""
        error_enum = None
        try:
            # throws a value error on failure
            error_enum = ErrorCodes(reply.rstrip('\r\n'))
            raise SyntaxError(f"Error. TigerController replied with error "
                              f"code: {str(error_enum)}.")
        except ValueError:
            pass

    @staticmethod
    def _reply_to_dict(reply):
        dict_reply = {}
        for line in reply.split('\r'):
            words = line.split(':')
            if len(words) == 2:
                val = words[1].split()
                dict_reply[words[0]] = val
        return dict_reply


if __name__ == '__main__':
    import pprint
    import time

    box = TigerController("COM10")

    TEST_TIME = 5 # seconds
    query_intervals = []
    print(f"running as many is_moving() queries in {TEST_TIME} seconds....")
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < TEST_TIME:
        box.move_axes_relative(z=11)
        fn_start_time = time.perf_counter()
        box.is_moving()
        fn_fin_time = time.perf_counter()
        query_intervals.append(fn_fin_time - fn_start_time)
    avg_time = sum(query_intervals)/len(query_intervals)
    print(f"average_time per query: {avg_time:.3f}")
