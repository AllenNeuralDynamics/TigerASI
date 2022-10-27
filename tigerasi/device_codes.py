#!/usr/bin/env python3
"""TigerController Device Codes"""
from enum import Enum


class Cmds(Enum):
    # Common commands in bytes form.
    BUILD_X = "BU X"
    HALT = "\\"
    STATUS = "/"
    RDSTAT = "RS" # RS [axis]?
    MOVEREL = "R"
    MOVEABS = "M"
    HOME = "!"
    HERE = "H"  # [axis]=0 [axis]=0
    WHERE = "W"  # [axis] [axis]
    BACKLASH = "B" # [axis]=0 [axis]=0
    CNTS = "CNTS" # [axis]?
    J = "J"  # Joystick
    JS = "JS"  # Joystick
    SCAN = "SCAN" # [X?] [Y=fast_axis_id, default X] [Z=slow_axis_id, default Y] [F=pattern]
    SCANR = "SCANR" # fast_axis [X=start in mm] [Y=stop in mm] [Z=enc_divide] [F= #_pixels] [R=retrace_speed]
    SCANV = "SCANV" # slow_axis [X=start in mm] [Y=stop in mm] [Z=number_of_lines] [F=overshoot_time in ms] [T=scan_overshoot]
    SETHOME = "HM"
    SETLOW = "SL"
    SETUP = "SU"
    SPEED = "S"
    TTL = "TTL" # [X=IN0_mode] [Y=OUT0_mode] [Z=aux_IO_state] [F=OUT0_polarity] [R=aux_IO_mask] [T=aux_IO_mode]
    PM = "PM" # PM [axis]=[0 or 1] for mirror, [0 or 3] for ETL
    PZINFO = "PZINFO" # [card address]PZINFO
    Z2B = "Z2B"  # Z2B Y? |or| Z2B Y=1 to set the axis id.


class ErrorCodes(Enum):
    # Error message responses from the Tiger Controller
    UNKNOWN_CMD = ':N-1'
    UNRECOGNIZED_AXIS_PARAMETER = ':N-2'
    MISSING_PARAMETERS = ':N-3'
    PARAMETER_OUT_OF_RANGE = ':N-4'
    OPERATION_FAILED = ':N-5'
    UNDEFINED_ERROR = ':N-6'
    INVALID_CARD_ADDRESS = ':N-7'
    RESERVED_8 = ':N-8'
    RESERVED_9 = ':N-9'
    RESERVED_10 = ':N-10'
    FILTERWHEEL_RESERVED_11 = ':N-11'
    FILTERWHEEL_RESERVED_12 = ':N-12'
    FILTERWHEEL_RESERVED_13 = ':N-13'
    FILTERWHEEL_RESERVED_14 = ':N-14'
    FILTERWHEEL_RESERVED_15 = ':N-15'
    FILTERWHEEL_RESERVED_16 = ':N-16'
    FILTERWHEEL_RESERVED_17 = ':N-17'
    FILTERWHEEL_RESERVED_18 = ':N-18'
    FILTERWHEEL_RESERVED_19 = ':N-19'
    FILTERWHEEL_RESERVED_20 = ':N-20'
    SERIAL_CMD_HALTED = ':N-21'


class JoystickInput(Enum):
    NONE = 0
    DEFAULT = 1
    JOYSTICK_X = 2  # default for x axis
    JOYSTICK_Y = 3  # default for y axis
    CONTROL_KNOB = 4  # default for z axis
    X_WHEEL = 5
    Y_WHEEL = 6
    ADC_CH1 = 7
    FOOTSWITCH = 8
    JX_X_WHEEL_COMBO = 9
    JY_Y_WHEEL_COMBO = 10
    CRIFF_KNOB = 11
    Z_WHEEL = 22
    F_WHEEL = 23



class ScanState(Enum):
    """Scan states"""
    # http://asiimaging.com/docs/commands/scan
    START = b'S'
    STOP = b'P'
    # More read-only scan states exist, but are not captured here.


class ScanPattern(Enum):
    """parameter for specifying scan pattern."""
    RASTER = 0
    SERPENTINE = 1


class ControlMode(Enum):
    INTERNAL_CLOSED_LOOP = 0
    EXTERNAL_CLOSED_LOOP = 1
    INTERNAL_OPEN_LOOP = 2
    EXTERNAL_OPEN_LOOP = 3
