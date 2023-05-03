"""TigerController Serial Port Abstraction"""
from enum import Enum
from serial import Serial, SerialException
from functools import cache, wraps
from time import sleep, perf_counter
from tigerasi.device_codes import *
from typing import Union
import logging

# Constants
STEPS_PER_UM = 10.0  # multiplication constant to convert micrometers to steps.
MM_SCALE = 4
DEG_SCALE = 3
DEFAULT_SPEED_PERCENT = 67.0
DEFAULT_SPEED_MM_PER_SEC = 7.68 * 67.0
REPLY_WAIT_TIME_S = 0.020  # minimum time to wait for a reply after having
                           # sent a command.
GET_INFO_STRING_SPLIT = 33 # index to split get info string reply


# Decorators
def axis_check(*args_to_skip: str):
    """Ensure that the axis (specified as an arg or kwd) exists.
    Additionally, sanitize all inputs to upper case.
    Parameters specified in the `args_to_skip` are omitted."""
    def wrap(func):
        # wraps needed for sphinx to make docs for methods with this decorator.
        @wraps(func)
        def inner(self, *args, **kwds):
            # Sanitize input to all-uppercase. Filter out specified parameters.
            args = [a.upper() for a in args if a not in args_to_skip]
            kwds = {k.upper(): v for k, v in kwds.items() if k not in args_to_skip}
            # Combine args and kwd names; skip double-adding params specified
            # as one or the other.
            iterable = [a for a in args if a not in kwds] + list(kwds.keys())
            for arg in iterable:
                assert arg.upper() in self.axes, \
                    f"Error. Axis '{arg.upper()}' does not exist"
            return func(self, *args, **kwds)
        return inner
    return wrap


def no_repeated_axis_check(func):
    """Ensure that an axis was specified either as an arg xor as a kwd."""

    @wraps(func)  # Required for sphinx doc generation.
    def inner(self, *args, **kwds):
        # Figure out if any axes was specified twice.
        intersection = {a.upper() for a in args} & \
                       {k.upper() for k, _ in kwds.items()}
        if len(intersection):
            raise SyntaxError("The following axes cannot be specified "
                              "both at the current position and at a specific "
                              f"position: {intersection}.")
        return func(self, *args, **kwds)
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
            logging.error("Error: could not open connection to Tiger "
                  "Controller. Is the device plugged in? Is another program "
                  "using it?")
            raise
        self._last_cmd_send_time = perf_counter()

        # Get the lettered axes in hardware order: ['X', 'Y', 'Z', ...].
        build_config = self.get_build_config()
        self.ordered_axes = build_config['Motor Axes']
        self.axis_to_card = self._get_axis_to_card_mapping(build_config)
        self.axis_to_type = self._get_axis_to_type_mapping(build_config)
        # Cache a list of firmware modules keyed by card address.
        self._card_modules = {self.axis_to_card[x][0]:
                              self._get_card_modules(self.axis_to_card[x][0])
                              for x in self.ordered_axes}
        ## FW-1000 filter wheels have their own command set but show up in
        # axis list as '0', '1' etc, so we remove them..
        self.ordered_filter_wheels = [fw for fw in self.ordered_axes if fw.isnumeric()]
        self.ordered_axes = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        # print(f"ordered axes are: {self.ordered_axes}")
        # Create O(1) lookup container.
        self.axes = set(self.ordered_axes)

        # Internal State Tracking to issue moves correctly.
        self._scan_card_addr = None  # card address on which the scan axes exist.
        self._scan_fast_axis = None
        self._array_scan_card_addr = None  # card address on which the array scan axes exist.
        self._last_rel_move_axes = []  # axes specified in previous MOVEREL
        self._rb_axes = []  # axes specified as movable by ring buffer moves.

    def halt(self, wait: bool = True):
        """stop any moving axis."""
        self._set_cmd_args_and_kwds(Cmds.HALT, wait=wait)

    # High-Level Commands
    @axis_check('wait')
    def move_relative(self, wait: bool = True, **axes: int):
        """Move the axes specified by a corresponding relative amount
        (in tenths of microns). Unspecified axes will not be moved.
        Implements `MOVEREL <http://asiimaging.com/docs/products/serial_commands#commandmovrel_r>`_ command.

        :param axes: one or more axes specified by name where the value is
            the relative position (in steps) to move to.

        .. code-block:: python

            box.move_relative(x=10, y=20)  # Move 1 micron in x and 2 in y
            box.move_relative(z=100)  # Move 10 microns in z

        """
        self._set_cmd_args_and_kwds(Cmds.MOVEREL, **axes, wait=wait)
        # Save the most recent MOVEREL axes to properly issue the TTL cmd.
        self._last_rel_move_axes = [x for x in axes if x in self.axes]

    @axis_check('wait')
    def move_absolute(self, wait: bool = True, **axes: int):
        """move the axes specified by a corresponding absolute amount.
        (in tenths of microns). Unspecified axes will not be moved.
        Implements `MOVE <http://asiimaging.com/docs/products/serial_commands#commandmove_m>`_ command.

        :param axes: one or more axes specified by name where the value is
            the absolute position (in steps) to move to.

        .. code-block:: python

            box.move_absolute(x=0, y=100)  # Move x and y axes to absolute location.

        """
        self._set_cmd_args_and_kwds(Cmds.MOVEABS, **axes, wait=wait)

    @axis_check('wait')
    def home(self, *axes: str, wait: bool = True):
        """Move to the preset home position (or hard axis travel limit) for
        the specified axes. If the preset position is not reachable, move until
        a hardware stage limit is reached.
        Implements `HOME <http://asiimaging.com/docs/products/serial_commands#commandhome>`_ command.

        Note: Because the homing procedure may either reach the specified
        software limit or a hardware limit, it is not safe to assume that
        a stage axis is in the prespecified homing position upon finishing
        this routine.

        .. code-block:: python

            box.home('x', 'y', 'z')  # home the specified axes.

        """
        self._set_cmd_args_and_kwds(Cmds.HOME, *axes, wait=wait)

    @axis_check('wait')
    @no_repeated_axis_check
    def set_home(self, *axes: str, wait: bool = True, **kwd_axes: float):
        """Set the current or specified position to home to in [mm].
        Implements `SETHOME <http://asiimaging.com/docs/products/serial_commands#commandsethome_hm>`_ command.

        Note: the values written here will persist across power cycles and
        adjust automatically such that the physical location remains constant.

        :param axes: axes for which to specify the current position as home.
        :param kwd_axes: axes for which to specify a particular position
            as home.
        :param wait: wait until the reply has been received.

        .. code-block:: python

            box.set_home('x', 'y', 'z')  # current position set as home OR
            box.set_home(x=100, y=20.5, z=0)  # specific positions for home OR
            box.set_home('x', y=20.5)  # mix of both.

        """
        args = [f"{ax}+" for ax in axes]
        return self._set_cmd_args_and_kwds(Cmds.SETHOME, *axes, **kwd_axes,
                                           wait=wait)

    @axis_check('wait')
    def reset_home(self, *axes: str, wait: bool = True):
        """Restore home values of the axes specified to firmware defaults.
        Implements `SETHOME <http://asiimaging.com/docs/products/serial_commands#commandsethome_hm>`_ command.

        Note: the firmware default is intentionally an unreachable stage
        position such that each axis triggers its hardware stage limit.
        """
        return self._reset_setting(Cmds.SETHOME, *axes, wait=wait)

    @axis_check()
    def get_home(self, *axes: str):
        """Return the position to home to in [mm] for the specified axes or all
        axes if none are specified.
        Implements `SETHOME <http://asiimaging.com/docs/products/serial_commands#commandsethome_hm>`_ command.

        Note: the returned value will adjust automatically such that the
        physical location remains constant.

        :param axes: the axes to get the machine frame home value for.
        """
        if not axes:  # Default to all axes if none are specified.
            axes = self.ordered_axes
        return self._get_axis_value(Cmds.SETHOME, *axes)

    @axis_check('wait')
    def zero_in_place(self, *axes: str, wait: bool = True):
        """Zero out the specified axes.
        (i.e: Set the specified axes current location to zero.)

        Note: the returned value will adjust automatically such that the
        physical location remains constant.

        .. code-block:: python

            box.zero_in_place('x', 'y')  # x and y axis' current locations are now zero.

        """
        # TODO: what happens if we home a device with CLOCKED POSITIONS?
        axis_positions = {}
        if not axes:  # Default to all lettered axes if none are specified.
            axes = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in axes:
            axis_positions[axis] = 0
        self.set_position(**axis_positions, wait=wait)

    @axis_check('wait')
    def set_position(self, wait: bool = True, **axes: float):
        """Set the specified axes to the specified positions.
        Similar to :meth:`zero_in_place`, but axes' current location can be
        specified to any location.

        :param axes: one or more axes specified by name where the value is
            the new absolute position (in steps).

        .. code-block:: python

            box.set_position(x=10, y=50)  # Current position is now (x=10, y=50).

        """
        self._set_cmd_args_and_kwds(Cmds.HERE, **axes, wait=wait)

    @axis_check('wait')
    @no_repeated_axis_check
    def set_lower_travel_limit(self, *axes: str, wait: bool = True,
                               **kwd_axes: float):
        """Set the specified axes lower travel limits to the current position
        or to a specified position in [mm].

        Note: the values written here will persist across power cycles and
        adjust automatically such that the physical location remains constant.

        :param axes: axes to specify the current position as lower limit.
        :param kwd_axes: axes to specify input position (in [mm]) as the lower
            limit.
        :param wait: wait until the reply has been received.

        .. code-block:: python

            box.set_lower_travel_limit('x', 'y')  # current positions as limit OR
            box.set_lower_travel_limit(x=50, y=4.0)  # specific positions as limit OR
            box.set_lower_travel_limit('x', y=20.5)  # mix of both.

        """
        axes = [f"{ax}+" for ax in axes]
        # Round axes values in mm to 4 decimal places.
        kwd_axes = {x: round(v, MM_SCALE) for x, v in kwd_axes.items()}
        return self._set_cmd_args_and_kwds(Cmds.SETLOW, *axes, **kwd_axes)

    def get_lower_travel_limit(self, *axes: str):
        """Get the specified axes' lower travel limits in [mm] as a dict.

        Note: the returned value will adjust automatically such that the
        physical location remains constant.

        Note: dict keys for lettered axes are uppercase.
        """
        return self._get_axis_value(Cmds.SETLOW, *axes)

    @axis_check('wait')
    def reset_lower_travel_limits(self, *axes: str, wait: bool = True):
        """Restore lower travel limit on specified axes (or all if none are
        specified) to firmware defaults."""
        if not axes:
            axes = self.ordered_axes
        return self._reset_setting(Cmds.SETLOW, *axes, wait=wait)

    @axis_check('wait')
    @no_repeated_axis_check
    def set_upper_travel_limit(self, *axes: str, wait: bool = True,
                               **kwd_axes: float):
        """Set the specified axes upper travel limits to the current position
        or to a specified position in [mm].

        Note: the values written here will persist across power cycles and
        adjust automatically such that the physical location remains
        constant.

        :param axes: axes to specify the current position as upper limit.
        :param kwd_axes: axes to specify input position as the upper limit.
        :param wait: wait until the reply has been received.

        .. code-block:: python

            box.set_upper_travel_limit('x', 'y')  # current positions as limit OR
            box.set_upper_travel_limit(x=50, y=4.0)  # specific positions as limit OR
            box.set_upper_travel_limit('x', y=20.5)  # mix of both.

        """
        axes = [f"{ax}+" for ax in axes]
        # Round axes values in mm to 4 decimal places.
        kwd_axes = {x: round(v, MM_SCALE) for x, v in kwd_axes.items()}
        return self._set_cmd_args_and_kwds(Cmds.SETUP, *axes, **kwd_axes)

    def get_upper_travel_limit(self, *axes: str):
        """Get the specified axes' upper travel limits in [mm] as a dict.

        Note: the returned value will adjust automatically such that the
        physical location remains constant.

        Note: dict keys for lettered axes are uppercase.
        """
        return self._get_axis_value(Cmds.SETUP, *axes)

    @axis_check('wait')
    def reset_upper_travel_limits(self, *axes: str, wait: bool = True):
        """Restore upper travel limit on specified axes (or all axes if none
         are specified) to firmware defaults."""
        if not axes:
            axes = self.ordered_axes
        return self._reset_setting(Cmds.SETUP, *axes, wait=wait)

    @axis_check('wait')
    def set_axis_backlash(self, wait: bool = True, **axes: float):
        """Set the backlash compensation value for one or more axes.
        Clear (i.e: disable) backlash compensation by writing 0 to that axis.
        A nonzero setting causes the corresponding axis to apply a backlash
        compensation routine where the axis oversteps by the specified amount
        such that the leadscrew is always being engaged from the same
        direction. The result is that a move will take ~25 extra milliseconds
        to complete.

        :param axes: one or more axes specified by name where the value is
            the absolute position (in steps) to move to.
        :param wait: wait until the reply has been received.
        """
        self._set_cmd_args_and_kwds(Cmds.BACKLASH, **axes, wait=wait)

    @axis_check()
    def get_position(self, *axes: str):
        """Return the controller's locations for lettered (non-numeric) axes.
        Note: filter wheel positions are not accessible this way.

        :param axes: one or more axes to request the current position from.

        :returns: a dict keyed by uppercase lettered axis whose value is
            the position (float).

        .. code-block:: python

            box.get_position('x')  # returns: {'X': 10}
            box.get_position('x', 'y')  # returns: {'X': 10, 'Y': 50}

        """
        axes_str = ""
        # Order the args since the hardware reply arrives in a fixed order.
        axes = self._order_axes(axes)
        if not axes:  # Default to all letter axes if none are specified.
            # Note: numeric (filter wheel) axes would be ignored if we added them.
            axes = [ax for ax in self.ordered_axes if not ax.isnumeric()]
        for axis in axes:
            axes_str += f" {axis.upper()}"
        cmd_str = Cmds.WHERE.value + axes_str + '\r'
        reply = self.send(cmd_str)
        axes_positions = [float(v) for v in reply.split()[1:]]
        return {k: v for k, v in zip(axes, axes_positions)}

    @axis_check('wait')
    def set_speed(self, wait: bool = True, **axes: float):
        """Set one or more axis speeds to a value in [mm/sec].
        Implements `SPEED <http://asiimaging.com/docs/products/serial_commands#commandspeed_s>`_ command.

        :param axes: one or more axes specified by name where the value is
            the speed in [mm/sec].
        :param wait: wait until the reply has been received.

        .. code-block:: python

            box.set_speed(x=50.5, y=10)

        """
        # Round axes values in mm to 4 decimal places.
        axes = {x: round(v, MM_SCALE) for x, v in axes.items()}
        self._set_cmd_args_and_kwds(Cmds.SPEED, **axes, wait=wait)

    # TODO: needs testing.
    @axis_check()
    def get_speed(self, *axes: str):
        """return the speed from the specified axis in [mm/s] or all axes if
        none are specified.

        :param axes: one or more lettered axes (case insensitive).
        :return: speed of requested axes in dict form (upper case).

        .. code-block:: python

            box.get_speed('x', 'z')  # returns: {'X': 50.5, 'Y': 10}

        """
        return self._get_axis_value(Cmds.SPEED, *axes)

    @axis_check()
    def bind_axis_to_joystick_input(self, **axes: JoystickInput):
        """Map a tiger axis to a joystick input. Implements
        `J <http://asiimaging.com/docs/products/serial_commands#commandjoystick_j>`_
        command.

        Note: binding a tigerbox stage axis to a joystick input does not affect
        the direction of the input. To change the direction, you must use the
        physical DIP switches on the back of the Tigerbox card.

        Note: binding a tigerbox stage axis to a joystick input `also` enables
        it.

        :param axes: one or more (case-insensitive) axes where the values are
            :obj:`~tigerasi.device_codes.JoystickInput` enums.

        .. code-block:: python

            from tigerasi.device_codes import JoystickInput

            box.bind_axis_to_joystick(x=JoystickInput.Y,
                                      y=JoystickInput.CONTROL_KNOB)

        """
        axes = {x: js_input.value for x, js_input in axes.items()}
        self._set_cmd_args_and_kwds(Cmds.J, **axes)

    @axis_check()
    def get_joystick_axis_mapping(self, *axes: str):
        """Get the axis mapping currently set on the joystick for the requested
        axes (or all if none are requested). Implements.
        `J <http://asiimaging.com/docs/products/serial_commands#commandjoystick_j>`_
        command.

        :return: a dict, keyed by (upper-case) axis, who's values are of type
            :obj:`~tigerasi.device_codes.JoystickInput` representing the
            assigned joystick input.
        """
        if not axes:
            axes = self.ordered_axes
        raw_dict = self._get_axis_value(Cmds.J, *axes)
        # Convert the reply codes (ints) to JoystickInput enums.
        return {x: JoystickInput(value) for x, value in raw_dict.items()}

    @axis_check()
    def set_joystick_axis_polarity(self, **axes: JoystickPolarity):
        """Set the joystick polarity of the axes specified. Implements
        `J <http://asiimaging.com/docs/products/serial_commands#commandjoystick_j>`_
        command.

        .. code-block:: python

            box.set_joystick_polarity(x=JoystickPolarity.DEFAULT,
                                      y=JoystickPolarity.INVERTED)

        """
        # Get axis mapping
        for axis_name, polarity in axes.items():
            # TODO: sanitize input within axis_check so we don't have to call 'upper'
            card_address, card_index = self.axis_to_card[axis_name.upper()]
            ccaz_value = 22 + polarity.value + card_index*2
            msg = f"{card_address}{Cmds.CCA.value} Z={ccaz_value}\r"
            self.send(msg)
        # Re-enable joystick inputs for the command to take effect.
        self.enable_joystick_inputs(*axes.keys())

    @axis_check()
    def enable_joystick_inputs(self, *axes: str):
        """Enable specified (or all if none are specified) axis control through
        the joystick. Implements
        `J <http://asiimaging.com/docs/products/serial_commands#commandjoystick_j>`_
        command.

        :param axes: one or more axes to re-enable joystick control for (or all
            if none are specified).

        .. code-block:: python

            box.enable_joystick_inputs('y')  # Enable joystick control of y axis.
            box.enable_joystick_inputs()  # Enable joystick control of all axes.

        Note: enabling joystick inputs will re-apply their default axis
        mapping. To disable/enable joystick inputs with a custom axis mapping,
        you must save the current axis mapping before disabling joystick
        inputs and then reapply it after enabling joystick inputs.

        ..code-block:: python

            axis_map = box.get_joystick_axis_mapping()  # Get current axis map for all axes.
            box.disable_joystick_inputs()  # Disable all joystick inputs.
            box.enable_joystick_inputs()  # Enable all joystick inputs. Default axis map is applied.
            box.bind_axis_to_joystick_input(**axis_map)  # Reapply original axis map for all axes.

        """
        if not axes:
            axes = self.ordered_axes
        enabled_axes = [f"{x.upper()}+" for x in axes]
        return self._set_cmd_args_and_kwds(Cmds.J, *enabled_axes)

    @axis_check()
    def disable_joystick_inputs(self, *axes: str):
        """Disable specified (or all if none are specified) axis control
        through the joystick. Implements
        `J <http://asiimaging.com/docs/products/serial_commands#commandjoystick_j>`_
        command.

        :param args: one or more axes to disable joystick control for (or all
            if none are specified).
        """
        if not axes:
            axes = self.ordered_axes
        disabled_axes = [f"{x.upper()}-" for x in axes]
        return self._set_cmd_args_and_kwds(Cmds.J, *disabled_axes)

    @axis_check()
    @cache
    def get_encoder_ticks_per_mm(self, axis: str):
        """Get <encoder ticks> / <mm of travel> for the specified axis.
        Implements `CNTS <http://asiimaging.com/docs/commands/CNTS>`_ command.
        """
        # TODO: can this function accept an arbitrary number of args?
        # FIXME: use _get_axis_value
        axis_str = f" {axis.upper()}?"
        cmd_str = Cmds.CNTS.value + axis_str + '\r'
        reply = self.send(cmd_str)
        return float(reply.split('=')[-1].split()[0])

    # TODO: consider making this function a hidden function that only gets
    #  called when a particular tigerbox command needs an axis specified by id.
    @axis_check()
    def get_axis_id(self, axis: str):
        """Get the hardware's axis id for a given axis.

        Note: some methods require that the axis is specified by id.

        :param axis: the axis of interest.
        :return: the axis id of the specified axis.
        """
        cmd_str = Cmds.Z2B.value + f" {axis.upper()}?" + '\r'
        reply = self.send(cmd_str)
        return int(reply.split('=')[-1])

    @axis_check('wait')
    def set_axis_control_mode(self, wait: bool = True,
                              **axes: Union[MicroMirrorControlMode,
                                            PiezoControlMode,
                                            TunableLensControlMode, int, str]):
        """Set an axis to a particular control mode.
        Implements `PM <http://asiimaging.com/docs/commands/pm>`_ command.

        Note: Setting an axis to external control enables control from the
        external TTL input port on the device hardware.

        :param axes: one or more axis control modes specified by key where the
            values are either a string, int, or one of these three enum types:
            :obj:`~tigerasi.device_codes.MicroMirrorControlMode`,
            :obj:`~tigerasi.device_codes.PiezoControlMode`, or
            :obj:`~tigerasi.device_codes.TunableLensControlMode`.
        :param wait: wait until the reply has been received.
        """
        axes = {x: v.value if isinstance(v, Enum) else str(v)
                for x, v in axes.items()}  # Convert keyword values to strings.

        self._set_cmd_args_and_kwds(Cmds.PM, **axes, wait=wait)

    @axis_check()
    def get_axis_control_mode(self, axis: str):
        """Get axis control mode. Implements
        `PM <http://asiimaging.com/docs/commands/pm>`_ command.

        :param axis: the axis of interest.
        :return: control mode (as a string) of the specified axis.
        """
        # example reply appears as 'V=1 :A'
        # assume control mode is a single digit
        control_num = str(int(self._get_axis_value(Cmds.PM, axis)[axis]))
        # TODO: figure out which axis type it is and return that type of enum.
        return control_num

    def setup_scan(self, fast_axis: str, slow_axis: str,
                   pattern: ScanPattern = ScanPattern.RASTER,
                   wait: bool = True):
        """setup scan pattern and define axes used for scanning.
        See ASI
        `SCAN Implementation <http://asiimaging.com/docs/products/serial_commands#commandscan_sn>`_
        for more details.

        Note: fast and slow scan axes must be on the same tiger card.

        :param fast_axis: the axis declared as the fast-scan axis.
        :param slow_axis: the axis declared as the slow-scan axis.
        :param pattern: :obj:`~tigerasi.device_codes.ScanPattern` enum.
            If unspecified, defaults to
            :obj:`~tigerasi.device_codes.ScanPattern.RASTER`.
        :param wait: wait until the reply has been received.
        """
        # Confirm that fast and slow axes are on the same card.
        cards = {self.axis_to_card[x][0]
                 for x in [fast_axis.upper(), slow_axis.upper()]}
        if len(cards) != 1:
            raise RuntimeError("Fast and slow axes must be on the same card.")
        self._scan_card_addr = cards.pop()
        self._scan_fast_axis = fast_axis
        # Firmware check.
        self._has_firmware(self._scan_card_addr, FirmwareModules.SCAN_MODULE)
        # Paramter setup.
        fast_axis_id = self.get_axis_id(fast_axis) if fast_axis is not None else None
        slow_axis_id = self.get_axis_id(slow_axis) if slow_axis is not None else None
        kwds = {}
        if fast_axis_id is not None:
            kwds['Y'] = fast_axis_id
        if slow_axis_id is not None:
            kwds['Z'] = slow_axis_id
        if pattern is not None:
            kwds['F'] = pattern.value
        self._set_cmd_args_and_kwds(Cmds.SCAN, **kwds, wait=wait,
                                    card_address=self._scan_card_addr)

    def scanr(self, scan_start_mm: float, pulse_interval_um: float,
              scan_stop_mm: float = None, num_pixels: int = None,
              retrace_speed_percent: int = DEFAULT_SPEED_PERCENT,
              wait: bool = True):
        """Setup the fast scanning axis start position and distance OR start
        position and number of pixels. To setup a scan, either scan_stop_mm
        or num_pixels must be specified, but not both.
        See ASI
        `SCANR Implementation <http://asiimaging.com/docs/commands/scanr>`_
        for more details.

        Note: :meth:`setup_scan` must be run first.

        :param scan_start_mm: absolute position to start the scan.
        :param pulse_interval_um: spacing (in [um]) between output pulses.
            i.e: a pulse will output every `pulse_interval_um`. Note that this
            value will be rounded to the nearest encoder tick. To set scan
            spacing to an exact encoder tick value, check
            :meth:`get_encoder_ticks_per_mm`. The logger will log a
            warning if the actual value in [um] was rounded.
        :param scan_stop_mm: absolute position to stop the scan. If
            unspecified, `num_pixels` is required.
        :param num_pixels:  number of pixels to output a pulse for. If
            unspecified, `scan_stop_mm` is required.
        :param retrace_speed_percent: percentage (0-100) of how fast to
            backtract to the scan start position after finishing a scan.
        :param wait: wait until the reply has been received.

        """

        # We can specify scan_stop_mm or num_pixels but not both (i.e: XOR).
        if not ((scan_stop_mm is None) ^ (num_pixels is None)):
            raise SyntaxError("Exclusively either scan_stop_mm or num_pixels "
                              "(i.e: one or the other, but not both) options "
                              "must be specified.")
        # Confirm that fast and slow axes have been defined.
        if self._scan_card_addr is None:
            raise RuntimeError("Cannot infer the card address for which to "
                               "apply the sttings. setup_scan must be run "
                               "first.")

        ENC_TICKS_PER_MM = self.get_encoder_ticks_per_mm(self._scan_fast_axis)
        pulse_interval_enc_ticks_f = ENC_TICKS_PER_MM * pulse_interval_um * 1e-3
        pulse_interval_enc_ticks = round(pulse_interval_enc_ticks_f)
        if pulse_interval_enc_ticks != pulse_interval_enc_ticks_f:
            rounded_pulse_interval_um = \
                pulse_interval_enc_ticks/(ENC_TICKS_PER_MM * 1e-3)
            self.log.warning(f"Requested scan {self._scan_fast_axis}-stack "
                           f"spacing: {pulse_interval_um:1f}[um]. Actual "
                           f"spacing: {rounded_pulse_interval_um:.1f}[um].")
        # Parameter setup.
        kwds = {
            'X': round(scan_start_mm, MM_SCALE),
            'Z': pulse_interval_enc_ticks}
        if scan_stop_mm is not None:
            kwds['Y'] = round(scan_stop_mm, MM_SCALE)
        if num_pixels is not None:
            kwds['F'] = num_pixels
        if retrace_speed_percent is not None:
            kwds['R'] = round(retrace_speed_percent)
        self._set_cmd_args_and_kwds(Cmds.SCANR, **kwds, wait=wait,
                                    card_address=self._scan_card_addr)

    def scanv(self, scan_start_mm: float, scan_stop_mm: float, line_count: int,
              overshoot_time_ms: int = None, overshoot_factor: float = None,
              wait: bool = True):
        """Setup the slow scanning axis.

        Behavior is equivalent to:
        ``numpy.linspace(scan_start_mm, scan_stop_mm, line_count, endpoint=False)``.
        See ASI
        `SCANV Implementation <http://asiimaging.com/docs/products/serial_commands#commandscanv_nv>`_
        for more details.

        Note: :meth:`setup_scan` must be run first.

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
        :param wait: wait until the reply has been received.
        """
        # Confirm that fast and slow axes have been defined.
        if self._scan_card_addr is None:
            raise RuntimeError("Cannot infer the card address for which to "
                               "apply the sttings. setup_scan must be run "
                               "first.")
        kwds = {
            'X': round(scan_start_mm, MM_SCALE),
            'Y': round(scan_stop_mm, MM_SCALE),
            'Z': line_count}
        if overshoot_time_ms is not None:
            kwds['F'] = round(overshoot_time_ms)
        if overshoot_factor is not None:
            kwds['T'] = round(overshoot_factor, MM_SCALE)
        self._set_cmd_args_and_kwds(Cmds.SCANV, **kwds, wait=wait,
                                    card_address=self._scan_card_addr)

    def start_scan(self, wait: bool = True):
        """Start a scan that has been previously setup with
        :meth:`scanr` :meth:`scanv` and :meth:`setup_scan`."""
        # Clear the card address for which the scan settings have been applied.
        # Use the previously specified card address.
        if self._scan_card_addr is None:
            raise RuntimeError("Cannot infer the card address for which to "
                               "apply the sttings. setup_scan must be "
                               "run first.")
        card_address = self._scan_card_addr
        # Clear card address for which the scan settings were specified.
        self._scan_card_addr = None
        self._scan_fast_axis = None
        self._set_cmd_args_and_kwds(Cmds.SCAN, ScanState.START.value,
                                    wait=wait, card_address=card_address)

    def stop_scan(self, wait: bool = True):
        """Stop an active scan."""
        self._set_cmd_args_and_kwds(Cmds.SCAN, ScanState.STOP.value, wait=wait)

    def setup_array_scan(self,
                         x_points: int = 0, delta_x_mm: float = 0,
                         y_points: int = 0, delta_y_mm: float = 0,
                         theta_deg: float = 0,
                         x_start_mm: int = None,
                         y_start_mm: int = None,
                         pattern: ScanPattern = ScanPattern.RASTER,
                         card_address: int = None,
                         wait: bool = True):
        """Configure Tiger-based grid-like array scan.
        See ASI
        `ARRAY Implementation <http://asiimaging.com/docs/products/serial_commands#commandarray_ar>`_
        and `supplement <https://asiimaging.com/docs/array>`_ for more details.

        Note: ASI docs recommend turning off backlash compensation on the scan
        axes, which can be done by setting compensation to zero (per axis) via
        the :meth:`set_axis_backlash` method.

        :param x_points: number of x points to visit including the start
            location. Zero if left unspecified.
        :param delta_x_mm: spacing (in [mm]) between movements. Zero if left
            unspecified.
        :param y_points: number of y points to visit including the start
            location. Zero if left unspecified.
        :param delta_y_mm: spacing (in [mm]) between movements. Zero if left
            unspecified.
        :param theta_deg: rotation from the x axis in [degrees] to rotate the
            array scan pivoting from the start position.
        :param x_start_mm: starting x axis location in [mm]. Current x position
            if left unspecified.
        :param y_start_mm: starting y axis location in [mm]. Current y position
            if left unspecified
        :param pattern: :obj:`~tigerasi.device_codes.ScanPattern` enum.
            If unspecified, defaults to
            :obj:`~tigerasi.device_codes.ScanPattern.RASTER`.
        :param card_address: The card hex address on which to specify the move.
            If unspecified, defaults to the only card with an x and y axis or
            throws a RuntimeError if multiple xy cards or no xy cards exist.
        :param wait: wait until the reply has been received.
        """
        # Infer address of the only card with an x and y axis if unspecified.
        if card_address is None:
            cards = {self.axis_to_card[x][0] for x in ['X', 'Y']}
            if len(cards) != 1:
                raise RuntimeError("Cannot infer the card address. It must be"
                                   "specified explicitly.")
            self._array_scan_card_addr = cards.pop()  # Get the only set item.
        else:
            self._array_scan_card_addr = card_address
        # Firmware check.
        self._has_firmware(self._array_scan_card_addr,
                           FirmwareModules.ARRAY_MODULE)
        # Specify scan pattern if specified.
        if pattern is not None:
            self._set_cmd_args_and_kwds(Cmds.SCAN, F=pattern.value, wait=wait,
                                        card_address=self._array_scan_card_addr)
        # Set start position.
        start_position = {}
        if x_start_mm is not None:
            start_position['X'] = round(x_start_mm, MM_SCALE)
        if y_start_mm is not None:
            start_position['Y'] = round(y_start_mm, MM_SCALE)
        self._set_cmd_args_and_kwds(Cmds.AHOME, **start_position, wait=wait,
                                    card_address=self._array_scan_card_addr)
        # Setup scan.
        scan_params = {
            'X': x_points,
            'Y': y_points,
            'Z': round(delta_x_mm, MM_SCALE),
            'F': round(delta_y_mm, MM_SCALE),
            'T': round(theta_deg, DEG_SCALE)}
        self._set_cmd_args_and_kwds(Cmds.ARRAY, **scan_params, wait=wait,
                                    card_address=self._array_scan_card_addr)

    def start_array_scan(self, wait: bool = True):
        """Start an array scan with parameters set by :meth:`setup_array_scan`.
        Note that this command is not needed if the scan is setup for external
        TTL pin triggering.

        Note: :meth:`setup_array_scan` must be run first.

        :param wait: wait until the reply has been received.
        """
        # Use the previously specified card address.
        if self._array_scan_card_addr is None:
            raise RuntimeError("Cannot infer the card address for which to "
                               "apply the sttings. setup_array_scan must be "
                               "run first.")
        card_address = self._array_scan_card_addr
        # Clear card address for which the array-scan settings were specified.
        self._array_scan_card_addr = None
        self._set_cmd_args_and_kwds(Cmds.ARRAY, card_address=card_address,
                                    wait=wait)

    def reset_ring_buffer(self, wait: bool = True):
        """Clear the ring buffer contents."""
        self._clear_ring_buffer(wait=wait)
        self._rb_axes = []

    def _clear_ring_buffer(self, wait: bool = True):
        """Clear the ring buffer contents."""
        kwds = {'X': 0}
        self._set_cmd_args_and_kwds(Cmds.RBMODE, **kwds, wait=wait)

    @axis_check('mode', 'wait')
    def setup_ring_buffer(self, *axes: str,
                          mode: RingBufferMode = RingBufferMode.TTL,
                          wait: bool = True):
        """Setup the ring buffer. Implements
        `RBMODE <https://asiimaging.com/docs/commands/rbmode>`_ command.

        :param axes: any number of axis names which will be enabled to move
            via moves queued into the ring buffer.
        :param mode: ring buffer mode specified as a
            :obj:`~tigerasi.device_codes.RingBufferMode` enum.
        :param wait: wait until the reply has been received.
        """
        # Enable the axes specified above to be movable from queued ring buffer
        # moves.
        axis_byte = 0
        for axis in axes:
            offset = self.ordered_axes.index(axis)
            axis_byte |= (1 << offset)
        axis_byte = axis_byte & 0xFFFF  # Fix axis byte to 32 bits wide.
        # Save axes so we can autoconfigure set_ttl_pin_modes without
        # specifying the card address.
        self._rb_axes = [x for x in axes]
        kwds = {'X': 0, 'Y': axis_byte, 'F': mode.value}  # X=0 clears ring buffer.
        self._set_cmd_args_and_kwds(Cmds.RBMODE, **kwds, wait=wait)

    @axis_check('wait')
    def queue_buffered_move(self, wait: bool = True, **axes: float,):
        """Push a move (relative or absolute depends on context) into the
        ring buffer.

        Note: if using TTL external input triggering, the TTL pin mode dictates
        whether the moves are absolute or relative. Mode can be set via:
        :meth:`set_ttl_pin_modes`.
        :obj:`~tigerasi.device_codes.TTLIn0Mode.MOVE_TO_NEXT_ABS_POSITION`
        will interpret stored moves in the buffer as absolute moves while
        :obj:`~tigerasi.device_codes.TTLIn0Mode.MOVE_TO_NEXT_REL_POSITION`
        will interpret stored moves in the buffer as relative moves.

        Note: the 'axis_byte' parameter must be set correctly such that the
        axes specified in the move are enabled to move.

        :param axes: one or more axes specified by name where the value is
            the absolute position (in steps) to move to.
        :param wait: wait until the reply has been received.
        """
        self._set_cmd_args_and_kwds(Cmds.LOAD, **axes, wait=wait)

    def set_ttl_pin_modes(self, in0_mode: TTLIn0Mode = None,
                          out0_mode: TTLOut0Mode = None,
                          reverse_output_polarity: bool = False,
                          aux_io_state: int = None,
                          aux_io_mask: int = None,
                          aux_io_mode: int = None,
                          card_address: int = None, wait: bool = True):
        """Setup ttl external IO modes or query the external output state
        (if the card specified without any additional arguments).

        See `ASI TTL Implementation <http://asiimaging.com/docs/commands/ttl>`_
        for more details.

        :param in0_mode: Set behavior of "IN" external TTL input pin.
            Optional if `in0_mode` is set to
            :obj:`~tigerasi.device_codes.TTLIn0Mode.REPEAT_LAST_MOVE`.
        :param out0_mode: Set behavior of "OUT" external TTL output pin.
            Optional if `in0_mode` is set to
            :obj:`~tigerasi.device_codes.TTLIn0Mode.REPEAT_LAST_MOVE`.
        :param reverse_output_polarity: bool. If True, output goes logic low
            when the output is asserted. Optional. Defaults to False.
        :param aux_io_state: Set to 0 if unused. Retains previous value if left
            unspecified. See ASI docs.
        :param aux_io_mask: Set to 0 if unused. Retains previous value if left
            unspecified. See ASI docs.
        :param aux_io_mode: Set what determines TTL value when set as outputs.
            Set to 0 if unused. Retains previous value if left unspecified.
            See ASI docs.
        :param card_address: The card hex address for which to apply the
            settings. Optional if `in0_mode` is set to
            :obj:`~tigerasi.device_codes.TTLIn0Mode.REPEAT_LAST_MOVE`,
            :obj:`~tigerasi.device_codes.TTLIn0Mode.ARRAY_MODE_MOVE_TO_NEXT_POSITION`,
            :obj:`~tigerasi.device_codes.TTLIn0Mode.MOVE_TO_NEXT_ABS_POSITION`,
            or
            :obj:`~tigerasi.device_codes.TTLIn0Mode.MOVE_TO_NEXT_REL_POSITION`.
        :param wait: wait until the reply has been received.

        .. code-block:: python

            from tiger_controller.device_codes import TTLIN0Mode as IN0Mode
            from tiger_controller.device_codes import TTLOUT0Mode as OUT0Mode

            # Make the input ttl pin repeat the last move.
            box.set_ttl_pin_modes(In0Mode.REPEAT_LAST_REL_MOVE,
                                  Out0Mode.PULSE_AFTER_MOVING,
                                  reverse_output_polarity=True)

            # OR: make the input ttl pin trigger a predefined ARRAY movement.
            box.set_ttl_pin_modes(IN0Mode.ARRAY_MODE_MOVE_TO_NEXT_POSITION,
                                  Out0Mode.PULSE_AFTER_MOVING,
                                  aux_io_state = 0, aux_io_mask = 0, aux_io_mode = 0)

        """
        in0_str = f" X={in0_mode.value} " if in0_mode is not None else ""
        out0_str = f" Y={out0_mode.value} " if out0_mode is not None else ""
        auxstate_str = f" Z={aux_io_state} " if aux_io_state is not None else ""
        polarity_str = f" F={-1 if reverse_output_polarity else 1} " \
            if reverse_output_polarity is not None else ""
        auxmask_str = f" R={aux_io_mask} " if aux_io_mask is not None else ""
        auxmode_str = f" T={aux_io_mode} " if aux_io_mode is not None else ""
        # Aggregate specified params.
        param_str = f"{in0_str}{out0_str}{auxstate_str}{polarity_str}" \
                    f"{auxmask_str}{auxmode_str}".rstrip()
        # Infer address of card or cards for a repeated move.
        if in0_mode == TTLIn0Mode.REPEAT_LAST_REL_MOVE and card_address is None:
            cards = {self.axis_to_card[x][0] for x in self._last_rel_move_axes}
            if not cards:
                raise RuntimeError("Cannot infer card address to configure "
                                   "device to repeat the last relative move "
                                   "when no previous relative move has been "
                                   "issued.")
            for card in cards:  # apply settings to each card.
                self._set_cmd_args_and_kwds(Cmds.TTL, param_str,
                                            card_address=card, wait=wait)
        # Infer card address(es) for ring buffer axis moves if it was setup.
        elif in0_mode in [TTLIn0Mode.MOVE_TO_NEXT_REL_POSITION,
                          TTLIn0Mode.MOVE_TO_NEXT_ABS_POSITION] \
                and card_address is None:
            if len(self._rb_axes) == 0:
                raise RuntimeError("Cannot infer the card address(es). "
                                   "Ring Buffer has not yet been setup.")
            cards = {self.axis_to_card[x][0] for x in self._rb_axes}
            for card in cards:  # apply settings to each card.
                self._set_cmd_args_and_kwds(Cmds.TTL, param_str,
                                            card_address=card, wait=wait)
        # Get the XY axis card for this setup since array scanning axes can't
        # be changed.
        elif in0_mode == TTLIn0Mode.ARRAY_MODE_MOVE_TO_NEXT_POSITION \
                and card_address is None:
            # Fetch card with the XY axes on it. Ensure there is only one.
            cards = {self.axis_to_card[x][0] for x in ['X', 'Y']}
            if len(cards) != 1:
                raise RuntimeError("Cannot infer the card address of the "
                                   "X and Y axes. card_address must be "
                                   "explicitly specified.")
            card = cards.pop()  # Get the only set item.
            self._has_firmware(card, FirmwareModules.ARRAY_MODULE)
            self._set_cmd_args_and_kwds(Cmds.TTL, param_str,
                                        card_address=card, wait=wait)
        # Default case: card address must be explicitly specified.
        else:
            if card_address is None:
                raise RuntimeError("Cannot infer the card address of the "
                                   "X and Y axes. card_address must be "
                                   "explicitly specified.")
            else:
                self._set_cmd_args_and_kwds(Cmds.TTL, param_str,
                                            card_address=card_address,
                                            wait=wait)

    def get_ttl_pin_modes(self, card_address: int, wait: bool = True):
        """Get the current TTL settings for a particular card."""
        param_query_str = "X? Y? Z? F? R? T?"
        return self._set_cmd_args_and_kwds(Cmds.TTL, param_query_str,
                                           card_address=card_address,
                                           wait=wait)

    def get_ttl_output_state(self, wait: bool = True):
        """Return the current state of the TTL output pin."""
        reply = self._set_cmd_args_and_kwds(Cmds.TTL, wait=wait)
        return bool(int(reply.lstrip(':A ')))

    def is_moving(self):
        """blocks. True if any axes is moving. False otherwise."""
        # Wait at least 20[ms] following the last time we sent a command.
        # (Handles edge case where the last command was sent with wait=False.)
        time_since_last_cmd = perf_counter() - self._last_cmd_send_time
        sleep_time = REPLY_WAIT_TIME_S - time_since_last_cmd
        if sleep_time > 0:
            sleep(sleep_time)
        # Send the inquiry.
        reply = self.send(f"{Cmds.STATUS.value}\r").rstrip('\r\n')
        # interpret reply.
        if reply == "B":
            return True
        elif reply == "N":
            return False
        else:
            raise RuntimeError(f"Error. Cannot tell if device is moving. "
                               f"Received: '{reply}'")

    def wait(self):
        """Block until tigerbox is idle."""
        while self.is_moving():
            pass

    def clear_incoming_message_queue(self):
        """Clear input buffer and reset skipped replies."""
        self.skipped_replies = 0
        self.ser.reset_input_buffer()

    # Low-Level Commands.
    def send(self, cmd_str: str, read_until: str = "\r\n", wait: bool = True):
        """Send a command; optionally wait for various conditions.
        :param cmd_str: command string with parameters and the proper line
            termination (usually '\r') to send to the tiger controller.
        :param read_until: the specific string to read until when reading back
            the response. (Default is fine for Tiger-based devices, but some
            filter wheels have a different response termination.)
        :param wait: wait until the reply has been received.
        """
        wait_for_output = wait  # wait for outgoing bytest to exit the PC.
        wait_for_reply = wait  # wait until at least one line has been read back.
        # TODO: clear input buffer before issuing a read-and-wait if the
        #  recv buffer is full. Use in_waiting.
        self.log.debug(f"Sending: {repr(cmd_str)}")
        self.ser.write(cmd_str.encode('ascii'))
        self._last_cmd_send_time = perf_counter()
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
            reply = \
                self.ser.read_until(read_until.encode("ascii")).decode("utf8")
            self.log.debug(f"Reply: {repr(reply)}")
            try:
                self.check_reply_for_errors(reply)
            except SyntaxError as e:  # Technically, this could be a skipped reply.
                self.log.error("Error occurred when sending: "
                               f"{repr(cmd_str)}")
                raise
            if self.skipped_replies:
                self.skipped_replies -= 1
            else:
                break
        return reply

    def get_info(self, axis: str):
        """Get the hardware's axis info for a given axis. Implements
        `INFO <https://asiimaging.com/docs/commands/info>`_ command.

        :param axis: the axis of interest.
        :return: the axis info of the specified axis.
        """
        cmd_str = Cmds.INFO.value + f" {axis.upper()}" + '\r'
        reply = self.send(cmd_str).strip('\r\n')
        # Reply is formatted in such a way that it can be put into dict form.
        # but reply is in lines with two columns
        # first column ends at index GET_INFO_STRING_SPLIT consistently
        dict_reply = {}
        for line in reply.split('\r'):
            cols = []
            cols.append(line[0:GET_INFO_STRING_SPLIT])
            cols.append(line[GET_INFO_STRING_SPLIT:len(line)])
            for col in cols:
                words = col.split(':')
                if len(words) == 2:  # skip eeprom
                    val = " ".join(words[1].split())  # remove redundant space.
                    dict_reply[words[0].strip(' ')] = val
        return dict_reply

    @axis_check('wait')
    def get_etl_temp(self, axis: str, wait: bool = True):
        """Get the etl temperature for a given axis.

        :param axis: the axis of interest.
        :return: etl temperature of the specified axis.
        """
        # enforce axis type for etl
        if self.axis_to_type[axis] != 'b':
            raise SyntaxError(f"Error. Axis '{axis}' is not an ETL")
        # get initial control mode
        ctrl_mode = TunableLensControlMode(self.get_axis_control_mode(axis))
        # must set to internal mode to read temperature. must wait for reply.
        self.set_axis_control_mode(
            **{axis: TunableLensControlMode.TG1000_INPUT_WITH_TEMP_COMPENSATION})
        # get pzinfo
        reply = self.get_pzinfo(self.axis_to_card[axis][0])
        # return control mode to initial value. must wait
        self.set_axis_control_mode(**{axis: ctrl_mode})
        # parse temperature from response
        # example line looks like:
        # 'V Mode[IN],Tc[21.250],TCOMP[ON]'
        for line in reply.split('\r'):
            if line.find('TCOMP[ON]') != -1:
                words = line.split(',')[1]
                temp = words[words.find('[')+1:words.find(']')]
        return temp

    def get_build_config(self):
        """return the configuration of the Tiger Controller.

        :return: a dict that looks like:

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

    @staticmethod
    def _get_axis_to_card_mapping(build_config: dict):
        """parse a build configuration dict to get axis-to-card relationship.

        :return: a dict that looks like
            ``{<axis>: (<hex_address>, <card_index>)), etc.}``

        .. code-block:: python

            # return type looks like:
            {'X': (31, 0),
             'Y': (31, 1)}

        """
        axis_to_card = {}
        curr_card_index = {c: 0 for c in set(build_config['Hex Addr'])}
        for axis, hex_addr in zip(build_config['Motor Axes'],
                                  build_config['Hex Addr']):
            card_index = curr_card_index[hex_addr]
            axis_to_card[axis] = (hex_addr, card_index)
            curr_card_index[hex_addr] = card_index + 1
        return axis_to_card

    @staticmethod
    def _get_axis_to_type_mapping(build_config: dict):
        """parse a build configuration dict to get axis-to-type relationship.

        :return: a dict that looks like
            ``{<axis>: <type>), etc.}``

        .. code-block:: python

            # return type looks like:
            {'X': 'X',
             'V': 'b'}

        """
        axis_to_type = {}
        curr_card_index = {c: 0 for c in set(build_config['Axis Types'])}
        for axis, axis_type in zip(build_config['Motor Axes'],
                                  build_config['Axis Types']):
            axis_to_type[axis] = axis_type
        return axis_to_type

    def get_pzinfo(self, card_address: int):
        """return the configuration of the specified card.

        :return: a dict
        """
        cmd_str = str(card_address) + Cmds.PZINFO.value + '\r'
        reply = self.send(cmd_str)
        return reply

    def _order_axes(self, axes: tuple[str]) -> list[str]:
        """return axes in the order they are received in replies from tigerbox.
        """
        axes = [ax.upper() for ax in axes]
        return [ax for ax in self.ordered_axes if ax in axes]

    def _reset_setting(self, cmd: Cmds, *args, wait: bool = True):
        """Reset a setting that takes an input query with specific syntax."""
        curr_pos_str = " ".join([f"{a.upper()}-" for a in args])
        cmd_str = f"{cmd.value} {curr_pos_str}\r"
        self.send(cmd_str, wait=wait)

    def _set_cmd_args_and_kwds(self, cmd: Cmds, *args: str, wait: bool = True,
                               card_address: int = None,
                               **kwds: Union[float, int]):
        """Flag a parameter or set a parameter with a specified value.

        .. code-block:: python

            box._set_cmd_args_and_kwds(Cmds.SETHOME, 'x', 'y', 'z')
            box._set_cmd_args_and_kwds(Cmds.SETHOME, 'x', y=10, z=20.5)
            box._set_cmd_args_and_kwds(Cmds.SETHOME, y=10, z=20.5)

        """
        card_addr_str = f"{card_address}" if card_address is not None else ""
        args_str = "".join([f" {a.upper()}" for a in args])
        kwds_str = "".join([f" {a.upper()}={v}" for a, v in kwds.items()])
        cmd_str = f"{card_addr_str}{cmd.value}{args_str}{kwds_str}\r"
        return self.send(cmd_str, wait=wait)

    def _get_axis_value(self, cmd: Cmds, *axes: str):
        """Get the value from one or more axes.
        This function creates a query string (ex: ``'HM X? Y?\r'``), sends it
        to the tigerbox, and returns the reply in dict form.

        :param cmd: a tigerbox command
        :param axes: one or more axis names (case-insensitive)
        """
        axes_str = " ".join([f"{a.upper()}?" for a in axes])
        cmd_str = f"{cmd.value} {axes_str}\r"
        # Trim the acknowledgement part of the response, which could show up at
        # the beginning or end, depending on the command.
        reply = self.send(cmd_str).rstrip("\r\n").strip(ACK).split()
        axis_val_tuples = [c.split("=") for c in reply]
        return {w[0].upper(): float(w[1]) for w in axis_val_tuples}

    def _get_card_modules(self, card_address):
        modules = []
        reply = self._set_cmd_args_and_kwds(Cmds.BUILD_X,
                                            card_address=card_address)
        for line in reply.split('\r'):
            # modules are specified in all-caps. Other lines can be ignored.
            if line != line.upper():
                continue
            modules.append(line)
        return modules

    def _has_firmware(self, card_address, *modules: FirmwareModules):
        """Raise RuntimeError if the specified card does not have the specified
        firmware.

        :param card_address: the card hex address.
        :param *modules: any number of modules specified as
            :obj:`~tigerasi.device_codes.FirmwareModules`.
        """
        missing_modules = []
        for module in modules:
            if module.value not in self._card_modules[card_address]:
                missing_modules.append(module.value)
        if len(missing_modules):
            raise RuntimeError(f"Error: card 0x{card_address} cannot execute "
                               f"the specified command because it is missing "
                               f"the following firmware modules: "
                               f"{missing_modules}")

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

