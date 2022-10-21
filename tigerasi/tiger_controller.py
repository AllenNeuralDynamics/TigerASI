#!/usr/bin/env python3
"""TigerController Serial Port Abstraction"""
from serial import Serial, SerialException
from functools import cache
from .device_codes import *
import logging

# Constants
UM_TO_STEPS = 10.0  # multiplication constant to convert micrometers to steps.


def axis_check(func):
    """Ensure that the axis (specified as an arg or kwarg) exists."""

    def inner(self, *args, **kwargs):
        # Check if axes are specified in *args.
        # Otherwise, check axes specified in **kwargs.
        iterable = args if len(args) else kwargs.keys()
        for arg in iterable:
            # skip keyworded wait flags.
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
    TIMEOUT = 1

    def __init__(self, com_port):
        self.ser = None
        self.log = logging.getLogger(__name__)
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
        # print(f"ordered axes are: {self.ordered_axes}")
        # Create O(1) lookup container.
        self.axes = set(self.ordered_axes)

    def halt(self, wait_for_output: bool = True, wait_for_reply: bool = True):
        """stop any moving axis."""
        self.send(Cmds.HALT, wait_for_output=wait_for_reply,
                  wait_for_reply=wait_for_reply)

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
    def set_axis_backlash(self, **kwargs: float):
        """Set the backlash compensation value.

        Note: clear backlash compensation by writing 0 to that axis.
        """
        axes_str = ""
        for axis, val in kwargs.items():
            axes_str += f" {axis.upper()}={round(val, 2)}"
        cmd_str = Cmds.BACKLASH.decode('utf8') + axes_str + '\r'
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

    @axis_check
    def set_speed(self, **kwargs: float):
        """Set one or more axis speeds to a value in [mm/sec]."""
        axes_str = ""
        for axis, speed in kwargs.items():
            axes_str += f" {axis.upper()}={round(speed, 4)}"
        cmd_str = Cmds.SPEED.decode('utf8') + axes_str + '\r'
        self.send(cmd_str.encode('ascii'))

    @axis_check
    def get_speed(self, axis: str):
        """return the speed from the specified axis in [mm/sec]."""
        axis_str = f" {axis.upper()}?"
        cmd_str = Cmds.SPEED.decode('utf8') + axis_str + '\r'
        reply = self.send(cmd_str.encode('ascii'))
        return float(reply.split('=')[-1])

    @axis_check
    @cache
    def get_encoder_ticks_per_mm(self, axis: str):
        """Get <encoder ticks> / <mm of travel> for the specified axis."""
        axis_str = f" {axis.upper()}?"
        cmd_str = Cmds.CNTS.decode('utf8') + axis_str + '\r'
        reply = self.send(cmd_str.encode('ascii'))
        return float(reply.split('=')[-1])

    @axis_check
    def get_axis_id(self, axis: str):
        """Get the hardware's axis id for a given axis."""
        cmd_str = Cmds.Z2B.decode('utf8') + f"{axis.upper()}?" + '\r'
        reply = self.send(cmd_str.encode('ascii'))
        return int(reply.split('=')[-1])

    @axis_check
    def pm(self, wait_for_output=True, wait_for_reply=True,
           **kwargs: ControlMode):
        """Set internal or external, open or closed loop, axis control.

        Setting an axis to external control enables control from the external
        TTL input port on the device hardware.
        """
        axes_str = ""
        for axis, ctrl_mode in kwargs.items():
            axes_str += f" {axis.upper()}={ctrl_mode.value}"
        cmd_str = Cmds.PM.decode('utf8') + axes_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def start_scan(self):
        self.scan(ScanState.START)

    def stop_scan(self):
        self.scan(ScanState.STOP)

    def scanr(self, scan_start_mm: float, scan_stop_mm: float = None,
              pulse_interval_enc_ticks: int = 1, num_pixels: int = None,
              retrace_speed: int = 67,
              wait_for_output: bool = True, wait_for_reply: bool = True):
        """Setup the fast scanning axis start position and distance OR start
        position and number of pixels. To setup a scan, either scan_stop_mm
        or num_pixels must be specified, but not both.

        See `ASI SCANR Implementation http://asiimaging.com/docs/commands/scanr`
        for more details.

        :param scan_start_mm: absolute position to start the scan.
        :param scan_stop_mm: absolute position to stop the scan. If
            unspecified, num_pixels is required.
        :param pulse_interval_enc_ticks: spacing (in encoder ticks) between
            output pulses.
            i.e: a pulse will output ever pulse_interval_enc_ticks.
        :param num_pixels:  number of pixels to output a pulse for. If
            unspecified, scan_stop_mm is required.
        :param retrace_speed: percentage (0-100) of how fast to backtrack to
            the scan start position after finishing a scan.
        :param wait_for_output: whether to wait for the message to exit the pc.
        :param wait_for_reply: whether to wait for the tigerbox to reply.
        """
        # We can specify scan_stop_mm or num_pixels but not both (i.e: XOR).
        if not ((scan_stop_mm is None) ^ (num_pixels is None)):
            raise SyntaxError("Exclusively either scan_stop_mm or num_pixels "
                              "(i.e: one or the other, but not both) options "
                              "must be specified.")
        # Build parameter list.
        scan_stop_str = f" Y={round(scan_stop_mm, 4)}" if scan_stop_mm else ""
        num_pixels_str = f" F={num_pixels}" if num_pixels else ""
        args_str = f" X={round(scan_start_mm, 4)}{scan_stop_str}" \
                   f" Z={pulse_interval_enc_ticks}{num_pixels_str}" \
                   f" R={retrace_speed}"
        cmd_str = Cmds.SCANR.decode('utf8') + args_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def scanv(self, scan_start_mm: float, scan_stop_mm: float, line_count: int,
              overshoot_time_ms: int = None, overshoot_factor: float = None,
              wait_for_output=True, wait_for_reply=True):
        """Setup the slow scanning axis.

        Behavior is equivalent to:
        :python:`numpy.linspace(scan_start_mm, scan_stop_mm, line_count, endpoint=False)`.

        See `ASI SCANV Implementation http://asiimaging.com/docs/commands/scanv`
        for more details.

        :param scan_start_mm: absolute position to start the scan in the slow
            axis dimension.
        :param scan_stop_mm: absolute position to stop the scan in the slow
            axis dimension.
        :param line_count: how many lines to scan on the slow axis.
        :param overshoot_time_ms: extra time (in ms) for the stage to settle
            (in addition to the current time set by the :code:`AC` command.)
        :param overshoot_factor: scalar multiplier (default: 1.0) to add
            distance to the start and stop of a scan before initiating the
            starting of pulses.
        :param wait_for_output: whether to wait for the message to exit the pc.
        :param wait_for_reply: whether to wait for the tigerbox to reply.
        """
        overshoot_time_str = f" F={overshoot_time_ms}" \
            if overshoot_time_ms is not None else ""
        overshoot_factor_str = f" T={round(overshoot_time_ms, 4)}" \
            if overshoot_time_ms is not None else ""
        args_str = f" X={round(scan_start_mm, 4)} Y={round(scan_stop_mm, 4)}" \
                   f" Z={line_count}{overshoot_time_str}{overshoot_factor_str}"
        cmd_str = Cmds.SCANV.decode('utf8') + args_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def scan(self, state: ScanState = None, fast_axis_id: str = None,
             slow_axis_id: str = None, pattern: ScanPattern = None,
             wait_for_output=True, wait_for_reply=True):
        """start scan and define axes used for scanning.

        Note: fast_axis and slow_axis are specified via 'axis id', which can
            be queried with the
            :meth:`~tiger_controller.TigerController.get_axis_id` query.

        :param state: start or stop the scan depending on input scan
            state.
        :param fast_axis: the axis (specified via axis id) declared as the
            fast-scan axis.
        :param slow_axis: the axis (specified via axis id) declared as the
            slow-scan axis.
        :param pattern: Raster or Serpentine scan pattern.
        :param wait_for_output: whether to wait for the message to exit the pc.
        :param wait_for_reply: whether to wait for the tigerbox to reply.
        """
        scan_state_str = f" {state.value}" if state is not None else ""
        fast_axis_str = f" Y={fast_axis_id}" if fast_axis_id is not None else ""
        slow_axis_str = f" Z={slow_axis_id}" if slow_axis_id is not None else ""
        pattern_str = f" F={pattern.value}" if pattern is not None else ""

        cmd_str = Cmds.SCAN.decode('utf8') + scan_state_str + fast_axis_str \
                  + slow_axis_str + pattern_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def ttl(self, in0_mode: int = None, out0_mode: int = None,
            reverse_output_polarity: bool = False,
            aux_io_state: int = None, aux_io_mask: int = None,
            aux_io_mode: int = None,
            wait_for_reply: bool = True, wait_for_output: bool = True):
        """Setup ttl external IO modes or query state (no arguments).
        
        See `ASI TTL Implementation http://asiimaging.com/docs/commands/ttl`
        for more details.

        :param in0_mode: set TTL trigger mode when configured as input.
        :param out0_mode:
        :param reverse_output_polarity:
        :param aux_io_state:
        :param aux_io_mask:
        :param aux_io_mode: Set what determines TTL value when set as outputs.
        :param wait_for_output: whether to wait for the message to exit the pc.
        :param wait_for_reply: whether to wait for the tigerbox to reply.
        """
        # TODO range checks.
        in0_str = f" X={in0_mode}" if in0_mode is not None else ""
        out0_str = f" Y={out0_mode}" if out0_mode is not None else ""
        auxstate_str = f" Z={aux_io_state}" if aux_io_state is not None else ""
        polarity_str = f" F={-1 if reverse_output_polarity else 1}" \
            if reverse_output_polarity is not None else ""
        auxmask_str = f" R={aux_io_mask}" if aux_io_mask is not None else ""
        auxmode_str = f" T={aux_io_mode}" if aux_io_mode is not None else ""
        # Aggregate specified params.
        param_str = f"{in0_str}{out0_str}{auxstate_str}{polarity_str}" \
                    f"{auxmask_str}{auxmode_str}"
        cmd_str = Cmds.TTL.decode('utf8') + param_str + '\r'
        self.send(cmd_str.encode('ascii'), wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

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
    def send(self, cmd_bytestr: bytes, wait_for_output=True, wait_for_reply=True):
        """Send a command; optionally wait for various conditions.

        :param wait: wait until the serial port finishes sending the message.
        :param wait_for_output: wait until all outgoing bytes exit the PC.
        :param wait_for_reply: wait until at least one line has been read in
                               by the PC.
        """
        self.log.debug(f"sending: {repr(cmd_bytestr.decode('utf8'))}")
        self.ser.write(cmd_bytestr)
        if wait_for_output:  # Wait for all bytes to exit the output buffer.
            while self.ser.out_waiting:
                pass
        # If we do not wait for a reply, we must track how many replies to read later.
        if not wait_for_reply:
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
            self.log.debug(f"reply: {repr(reply)}")
            try:
                self.check_reply_for_errors(reply)
            except SyntaxError as e:  # Technically, this could be a skipped reply.
                self.log.error("Error occurred when sending: "
                               f"{repr(cmd_bytestr)}")
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

    def get_pzinfo(self, card_address):
        """return the configuration of the specified card.

        returns: a dict
        """
        cmd_str = str(card_address) + Cmds.PZINFO.decode('utf8') + '\r'
        reply = self.send(cmd_str.encode('ascii'))
        # note: reply is not formatted to dict
        return self._reply_split(reply)

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

    TEST_TIME = 5  # seconds
    query_intervals = []
    print(f"running as many is_moving() queries in {TEST_TIME} seconds....")
    start_time = time.perf_counter()
    while time.perf_counter() - start_time < TEST_TIME:
        box.move_axes_relative(z=11)
        fn_start_time = time.perf_counter()
        box.is_moving()
        fn_fin_time = time.perf_counter()
        query_intervals.append(fn_fin_time - fn_start_time)
    avg_time = sum(query_intervals) / len(query_intervals)
    print(f"average_time per query: {avg_time:.3f}")
