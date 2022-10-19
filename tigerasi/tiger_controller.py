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

    @wraps(func)  # Required for sphinx doc generation.
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


def no_repeated_axis_check(func):
    """Ensure that an axis was specified either as an arg xor as a kwarg."""
    def inner(self, *args, **kwargs):
        # Figure out if any axes was specified twice.
        intersection = {a.upper() for a in args} & \
                       {k.upper() for k, _ in kwargs.items()}
        if len(intersection):
            raise SyntaxError("The following axes cannot be specified "
                              "both at the current position and at a specific "
                              f"position: {intersection}.")
        return func(self, *args, **kwargs)
    return inner


class TigerController:
    """Tiger Box Serial Port Abstraction."""

    # Constants
    BAUD_RATE = 115200
    TIMEOUT = 1

    def __init__(self, com_port: str):
        """Init. Creates serial port connection and connects to hardware.

        :param com_port: serial com port.

        .. code-block:: python

            box = TigerController('COM4')

        """
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

        # Get the lettered axes in hardware order: ['X', 'Y', 'Z', ...].
        self.ordered_axes = self.get_build_config()['Motor Axes']
        ## FW-1000 filter wheels have their own command set but show up in
        # axis list as '0', '1' etc, so we remove them..
        self.ordered_filter_wheels = [fw for fw in self.ordered_axes if fw.isnumeric()]
        self.ordered_axes = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        #print(f"ordered axes are: {self.ordered_axes}")
        # Create O(1) lookup container.
        self.axes = set(self.ordered_axes)

    def halt(self, wait_for_output: bool = True, wait_for_reply: bool = True):
        """stop any moving axis."""
        self.send(f"{Cmds.HALT.value}\r", wait_for_output=wait_for_reply,
                  wait_for_reply=wait_for_reply)

    # High-Level Commands
    @axis_check
    def move_axes_relative(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """Move the axes specified in kwargs by a relative amount.

        Note: Units are in tenths of microns.

        :param kwargs: one or more axes specified by name where the value is
            the relative position (in steps) to move to.

        .. code-block:: python

            box.move_axis_relative(x=10, y=20)  # Move 1 micron in x and 2 in y
            box.move_axis_relative(z=100)  # Move 10 microns in z

        """
        self._set_cmd_args_and_kwds(Cmds.MOVEREL, **kwargs,
                                    wait_for_output=wait_for_output,
                                    wait_for_reply=wait_for_reply)

    @axis_check
    def move_axes_absolute(self, wait_for_output=True, wait_for_reply=True,
                           **kwargs: int):
        """move the axes specified in kwargs by the specified absolute amount
        (in tenths of microns). Unspecified axes will not be moved.

        :param kwargs: one or more axes specified by name where the value is
            the absolute position (in steps) to move to.

        .. code-block:: python

            box.move_axis_absolute(x=0, y=100)  # Move x and y axes to absolute location.

        """
        self._set_cmd_args_and_kwds(Cmds.MOVEABS, **kwargs,
                                    wait_for_output=wait_for_output,
                                    wait_for_reply=wait_for_reply)

    @axis_check
    def home(self, *args: str,
             wait_for_output: bool = True, wait_for_reply: bool = True):
        """Move to the preset home position (or hard axis travel limit) for
        the specified axes. If the preset position is not reachable, move until
        a hardware stage limit is reached.

        Note: Because the homing procedure may either reach the specified
            software limit or a hardware limit, it is not safe to assume that
            a stage axis is in the prespecified homing position upon finishing
            this routine.
        """
        self._set_cmd_args_and_kwds(self.HOME, *args,
                                    wait_for_output=wait_for_output,
                                    wait_for_reply=wait_for_reply)

    @axis_check
    @no_repeated_axis_check
    def set_home(self, *args: str, wait_for_output: bool = True,
                 wait_for_reply: bool = True, **kwargs: float):
        """Set the current or specified position to home to in [mm].

        Note: the values written here will persist across power cycles and
            adjust automatically such that the physical location remains
            constant.

        :param args: axes for which to specify the current position as home.
        :param kwargs: axes for which to specify a particular position as home.
        :param wait_for_output: wait until all outgoing bytes exit the PC.
        :param wait_for_reply: wait until at least one line has been read in
                               by the PC.

        ..code_block::
            set_home('x', 'y', 'z')  # current position set as home OR
            set_home(x=100, y=20.5, z=0)  # specific positions for home OR
            set_home('x', y=20.5)  # mix of both.
        """
        args = [f"{ax}+" for ax in args]
        return self._set_cmd_args_and_kwds(Cmds.SETHOME, *args, **kwargs,
                                           wait_for_output=wait_for_output,
                                           wait_for_reply=wait_for_reply)

    @axis_check
    def reset_home(self, *args: str, wait_for_output: bool = True,
                   wait_for_reply: bool = True):
        """Restore home values of the axes specified to firmware defaults.

        Note: the firmware default is intentionally an unreachable stage
        position such that each axis triggers its hardware stage limit.
        """
        return self._reset_setting(Cmds.SETHOME, *args,
                                   wait_for_output=wait_for_output,
                                   wait_for_reply=wait_for_reply)

    @axis_check
    def get_home(self, *args: str):
        """Return the position to home to in [mm] for the specified axes.

        Note: the returned value will adjust automatically such that the
            physical location remains constant.

        :param args: the axes to get the machine frame home value for.
        """
        return self._get_axis_value(Cmds.SETHOME, *args)

    @axis_check
    def zero_in_place(self, *args: str):
        """Zero out the specified axes.
        (i.e: Set the specified axes current location to zero.)

        Note: the returned value will adjust automatically such that the
            physical location remains constant.

        .. code-block:: python

            box.home_in_place('x', 'y')  # x and y axis' current locations are now zero.

        """
        # TODO: what happens if we home a device with CLOCKED POSITIONS?
        axis_positions = {}
        if not args:
            # Default to all lettered axes.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in args:
            axis_positions[axis] = 0
        self.set_position(**axis_positions)

    @axis_check
    def set_position(self, wait_for_output: bool = True,
                     wait_for_reply: bool = True, **kwargs: float):
        """Set the specified axes to the specified positions.
        Similar to :meth:`home_in_place`, but axes' current location can be
        specified to any location.

        :param kwargs: one or more axes specified by name where the value is
            the new absolute position (in steps).

        .. code-block:: python

            box.set_position(x=10, y=50)  # Current position is now (x=10, y=50).

        """
        self._set_cmd_args_and_kwds(Cmds.HERE, **kwargs,
                                    wait_for_output=wait_for_output,
                                    wait_for_reply=wait_for_reply)

    @axis_check
    @no_repeated_axis_check
    def set_lower_travel_limit(self, *args: str, wait_for_output: bool = True,
                               wait_for_reply: bool = True, **kwargs: float):
        """Set the specified axes lower travel limits to the current position
        or to a specified position in [mm].

        Note: the values written here will persist across power cycles and
            adjust automatically such that the physical location remains
            constant.

        :param args: axes to specify the current position as lower limit.
        :param kwargs: axes to specify input position as the lower limit.
        :param wait_for_output: wait until all outgoing bytes exit the PC.
        :param wait_for_reply: wait until at least one line has been read in
                               by the PC.

        .. code-block:: python
            set_lower_travel_limit('x', 'y')  # current positions as limit OR
            set_lower_travel_limit(x=50, y=4.0)  # specific positions as limit OR
            set_lower_travel_limit('x', y=20.5)  # mix of both.
        """
        args = [f"{ax}+" for ax in args]
        return self._set_cmd_args_and_kwds(Cmds.SETLOW, *args, **kwargs,
                                           wait_for_output=wait_for_output,
                                           wait_for_reply=wait_for_reply)

    @axis_check
    def get_lower_travel_limit(self, *args: str):
        """Get the specified axes travel limits as a dict.

        Note: the returned value will adjust automatically such that the
            physical location remains constant.
        Note: dict keys for lettered axes are uppercase.
        """
        return self._get_axis_value(Cmds.SETLOW, *args)

    @axis_check
    def reset_lower_travel_limits(self, *args: str,
                                  wait_for_output: bool = True,
                                  wait_for_reply: bool = True):
        """Restore lower travel limit on specified axes to firmware defaults."""
        return self._reset_setting(Cmds.SETLOW, *args,
                                   wait_for_output=wait_for_output,
                                   wait_for_reply=wait_for_reply)

    @axis_check
    @no_repeated_axis_check
    def set_upper_travel_limit(self, *args: str, wait_for_output: bool = True,
                               wait_for_reply: bool = True, **kwargs: float):
        """Set the specified axes upper travel limits to the current position
        or to a specified position in [mm].

        Note: the values written here will persist across power cycles and
            adjust automatically such that the physical location remains
            constant.

        :param args: axes to specify the current position as upper limit.
        :param kwargs: axes to specify input position as the upper limit.
        :param wait_for_output: wait until all outgoing bytes exit the PC.
        :param wait_for_reply: wait until at least one line has been read in
                               by the PC.

        ..code_block::
            set_upper_travel_limit('x', 'y')  # current positions as limit OR
            set_upper_travel_limit(x=50, y=4.0)  # specific positions as limit OR
            set_upper_travel_limit('x', y=20.5)  # mix of both.
        """
        args = [f"{ax}+" for ax in args]
        return self._set_cmd_args_and_kwds(Cmds.SETUP, *args, **kwargs,
                                           wait_for_output=wait_for_output,
                                           wait_for_reply=wait_for_reply)

    @axis_check
    def get_upper_travel_limit(self, *args: str):
        """Get the specified upper axes travel limits as a dict.

        Note: the returned value will adjust automatically such that the
            physical location remains constant.
        Note: dict keys for lettered axes are uppercase.
        """
        return self._get_axis_value(Cmds.SETUP, *args)

    @axis_check
    def reset_upper_travel_limits(self, *args: str,
                                  wait_for_output: bool = True,
                                  wait_for_reply: bool = True):
        """Restore lower travel limit on specified axes to firmware defaults."""
        return self._reset_setting(Cmds.SETUP, *args,
                                   wait_for_output=wait_for_output,
                                   wait_for_reply=wait_for_reply)

    def set_position(self, **kwargs: float):
        """Set the specified axes to the specified positions.
        Similar to :meth:`home_in_place`, but axes' current location can be
        specified to any location.

        :param kwargs: one or more axes specified by name where the value is
            the new absolute position (in steps).

        .. code-block:: python

            box.set_position(x=10, y=50)  # Current position is now (x=10, y=50).

        """
        axes_str = ""
        for axis, val in kwargs.items():
            axes_str += f" {axis.upper()}={val}"
        cmd_str = Cmds.HERE.decode('utf8') + axes_str + '\r'
        self.send(cmd_str.encode('ascii'))

    @axis_check
    def set_axis_backlash(self, **kwargs: float):
        """Set the backlash compensation value for one or more axes.
        Clear (i.e: disable) backlash compensation by writing 0 to that axis.
        A nonzero setting causes the corresponding axis to apply a backlash
        compensation routine where the axis oversteps by the specified amount
        such that the leadscrew is always being engaged from the same
        direction. The result is that a move will take ~25 extra milliseconds
        to complete.

        :param kwargs: one or more axes specified by name where the value is
            the absolute position (in steps) to move to.
        """
        self._set_cmd_args_and_kwds(Cmds.BACKLASH, **kwargs,
                                    wait_for_output=wait_for_output,
                                    wait_for_reply=wait_for_reply)

    @axis_check
    def get_position(self, *args: str):
        """Return the controller's locations for lettered (non-numeric) axes.
        Note: filter wheel positions are not accessible this way.

        :param: one or more axes to request the current position from.

        :returns: a dict keyed by uppercase lettered axis whose value is
            the position (float).

        .. code-block:: python

            box.get_position('x')  # returns: {'X': 10}
            box.get_position('x', 'y')  # returns: {'X': 10, 'Y': 50}

        """
        axes_str = ""
        # Order the args since the hardware reply arrives in a fixed order.
        args = self._order_axes(args)
        # Fill out all args if none are populated.
        if not args:
            # Default to all lettered axes.
            # Note: numeric (filter wheel) axes would be ignored if we added them.
            args = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in args:
            axes_str += f" {axis.upper()}"
        cmd_str = Cmds.WHERE.value + axes_str + '\r'
        reply = self.send(cmd_str)
        axes_positions = [float(v) for v in reply.split()[1:]]
        return {k: v for k, v in zip(args, axes_positions)}

    @axis_check
    def set_speed(self, wait_for_output: bool = True,
                  wait_for_reply: bool = True, **kwargs: float):
        """Set one or more axis speeds to a value in [mm/sec].

        :param kwargs: one or more axes specified by name where the value is
            the speed in [mm/sec].

        .. code-block:: python

            box.set_speed(x=50.5)
        """
        self._set_cmd_args_and_kwds(Cmds.SPEED, **kwargs,
                                    wait_for_output=wait_for_output,
                                    wait_for_reply=wait_for_reply)

    # TODO: needs testing.
    @axis_check
    def get_speed(self, *args):
        """return the speed from the specified axis in [mm/sec]."""
        return self._get_axis_value(Cmds.SPEED, *args)

    @axis_check
    @cache
    def get_encoder_ticks_per_mm(self, axis: str):
        """Get <encoder ticks> / <mm of travel> for the specified axis."""
        # TODO: can this function accept an arbitrary number of args?
        axis_str = f" {axis.upper()}?"
        cmd_str = Cmds.CNTS.value + axis_str + '\r'
        reply = self.send(cmd_str)
        return float(reply.split('=')[-1])

    # TODO: consider making this function a hidden function that only gets
    #  called when a particular tigerbox command needs an axis specified by id.
    @axis_check
    def get_axis_id(self, axis: str):
        """Get the hardware's axis id for a given axis.

        Note: some methods require that the axis is specified by id.

        :param axis: the axis of interest.
        :return: the axis id of the specified axis.
        """
        cmd_str = Cmds.Z2B.decode('utf8') + f"{axis.upper()}?" + '\r'
        reply = self.send(cmd_str.encode('ascii'))
        return int(reply.split('=')[-1])

    @axis_check
    def pm(self, wait_for_output=True, wait_for_reply=True,
           **kwargs: ControlMode):
        """Set internal or external, open or closed loop, axis control.

        Setting an axis to external control enables control from the external
        TTL input port on the device hardware.

        :param kwargs: one or more axes specified by key where the values are
            :obj:`~tigerasi.device_codes.ControlMode` enums.

        """
        axes_str = ""
        for key, val in kwargs.items():
            axes_str += f" {key.upper()}={val}"
        cmd_str = Cmds.PM.value + axes_str + '\r'
        self.send(cmd_str, wait_for_output=wait_for_output,
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

        See ASI
        `SCANR Implementation <http://asiimaging.com/docs/commands/scanr>`_
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
        scan_stop_str = f" Y={round(scan_stop, 4)}" if scan_stop else ""
        num_pixels_str = f" F={pixels}" if num_pixels else ""
        args_str = f" X={round(scan_start_mm, 4)}{scan_stop_str}" \
                   f" Z={pulse_interval_enc_ticks}{num_pixels_str}" \
                   f" R={retrace_speed}"
        cmd_str = Cmds.SCANR.value + args_str + '\r'
        self.send(cmd_str, wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def scanv(self, scan_start_mm: float, scan_stop_mm: float, line_count: int,
              overshoot_time_ms: int = None, overshoot_factor: float = None,
              wait_for_output=True, wait_for_reply=True):
        """Setup the slow scanning axis.

        Behavior is equivalent to:
        :python:``numpy.linspace(scan_start_mm, scan_stop_mm, line_count, endpoint=False)``.

        See ASI
        `SCANV Implementation <http://asiimaging.com/docs/commands/scanv>`_
        for more details.

        :param scan_start_mm: absolute position to start the scan in the slow
            axis dimension.
        :param scan_stop_mm: absolute position to stop the scan in the slow
            axis dimension.
        :param line_count: how many lines to scan on the slow axis.
        :param overshoot_time_ms: extra time (in ms) for the stage to settle
            (in addition to the current time set by the ``AC`` command.)
        :param overshoot_factor: scalar multiplier (default: 1.0) to add
            distance to the start and stop of a scan before initiating the
            starting of pulses.
        :param wait_for_output: whether to wait for the message to exit the pc.
        :param wait_for_reply: whether to wait for the tigerbox to reply.
        """
        overshoot_time_str = f" F={overshoot_time_ms}" \
            if overshoot_time_ms is not None else ""
        overshoot_factor_str = f" T={round(overshoot_factor_mm, 4)}" \
            if overshoot_factor_mm is not None else ""
        args_str = f" X={round(scan_start_mm, 4)} Y={round(scan_stop_mm, 4)}" \
                   f" Z={line_count}{overshoot_time_str}{overshoot_factor_str}"
        cmd_str = Cmds.SCANV.value + args_str + '\r'
        self.send(cmd_str.value, wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def scan(self, state: ScanState = None, fast_axis_id: str = None,
             slow_axis_id: str = None, pattern: ScanPattern = None):
        """start scan and define axes used for scanning.

        Note: fast_axis and slow_axis are specified via 'axis id', which can
            be queried with the
            :meth:`~tiger_controller.TigerController.get_axis_id` query.

        :param state: start or stop the scan depending on input scan
            state.
        :param fast_axis_id: the axis (specified via axis id) declared as the
            fast-scan axis.
        :param slow_axis_id: the axis (specified via axis id) declared as the
            slow-scan axis.
        :param pattern: Raster or Serpentine scan pattern.
        """
        scan_state_str = f" {state.value}" if state is not None else ""
        fast_axis_str = f" Y={fast_axis}" if fast_axis is not None else ""
        slow_axis_str = f" Z={slow_axis}" if slow_axis is not None else ""
        pattern_str = f" F={pattern.value}" if pattern is not None else ""

        cmd_str = Cmds.SCAN.value + scan_state_str + fast_axis_str \
                  + slow_axis_str + pattern_str + '\r'
        self.send(cmd_str, wait_for_output=wait_for_output,
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
        cmd_str = Cmds.TTL.value + param_str + '\r'
        self.send(cmd_str, wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def is_moving(self):
        """blocks. True if any axes is moving. False otherwise."""
        # Send the inquiry.
        reply = self.send(Cmds.STATUS.value).rstrip('\r\n')
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
    def send(self, cmd_str: str, wait_for_output=True, wait_for_reply=True):
        """Send a command; optionally wait for various conditions.

        :param cmd_str: command string with parameters and terminated with '\r'
            to send to the tiger controller.
        :param wait_for_output: wait until all outgoing bytes exit the PC.
        :param wait_for_reply: wait until at least one line has been read in
                               by the PC.
        """
        self.log.debug(f"sending: {cmd_str}")
        self.ser.write(cmd_str.encode('ascii'))
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

        :returns: a dict that looks like:

        .. code-block:: python

            {'Axis Addr': [],
             'Axis Props': ['74', '10', '2', etc.], # these are positions
             'Axis Types': ['x', 'x', 'z', etc],
             'Hex Addr': [],
             'Motor Axes': ['X', 'Y', 'Z', etc]}

        """
        reply = self.send(f"{Cmds.BUILD_X.value}\r")
        # Reply is formatted in such a way that it can be put into dict form.
        return self._reply_to_dict(reply)

    def get_pzinfo(self, card_address):
        """return the configuration of the specified card.

        returns: a dict
        """
        cmd_str = str(card_address) + Cmds.PZINFO.value + '\r'
        reply = self.send(cmd_str)
        # note: reply is not formatted to dict
        return self._reply_split(reply)

    def _order_axes(self, axes: tuple[str]) -> list[str]:
        """return axes in the order they are received in replies from tigerbox.
        """
        axes = [ax.upper() for ax in axes]
        return [ax for ax in self.ordered_axes if ax in axes]

    def _reset_setting(self, cmd: Cmds, *args, wait_for_output: bool = True,
                       wait_for_reply: bool = True):
        """Reset a setting that takes an input query with specific syntax."""
        curr_pos_str = " ".join([f"{a.upper()}-" for a in args])
        cmd_str = f"{cmd.value} {curr_pos_str}\r"
        self.send(cmd_str, wait_for_output=wait_for_output,
                  wait_for_reply=wait_for_reply)

    def _set_cmd_args_and_kwds(self, cmd: Cmds, *args: str,
                               wait_for_output: bool = True,
                               wait_for_reply: bool = True, **kwargs: float):
        """Flag a parameter or set a parameter with a specified value.

        ..code::
            _set_cmd_args_and_kwds(Cmds.SETHOME, 'x', 'y', 'z')
            _set_cmd_args_and_kwds(Cmds.SETHOME, 'x', y=10, z=20.5)
            _set_cmd_args_and_kwds(Cmds.SETHOME, y=10, z=20.5)
        """
        args_str = " ".join([f"{a.upper()}" for a in args])
        kwds_str = " ".join([f"{a.upper()}={v}" for a, v in kwargs.items()])
        cmd_str = f"{cmd.value} {args_str} {kwds_str}\r"
        return self.send(cmd_str, wait_for_output=wait_for_output,
                         wait_for_reply=wait_for_reply)

    def _get_axis_value(self, cmd: Cmds, *args):
        """Get the value from one or more axes."""
        axes_str = " ".join([f"{a.upper()}?" for a in args])
        cmd_str = f"{cmd.value} {axes_str}\r"
        reply = self.send(cmd_str).split()[1:]  # Trim the acknowledgement part
        axis_val_tuples = [c.split("=") for c in reply]
        return {w[0].upper(): float(w[1]) for w in axis_val_tuples}

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
