#!/usr/bin/env python3
"""TigerController Device Codes"""
from enum import Enum

ACK = ":A"  # device acknowledgment for some axis-specific commands.

class Cmds(Enum):
    # Common commands in bytes form.
    BUILD_X = "BU X"
    CCA = "CCA"
    HALT = "\\"
    STATUS = "/"
    RDSTAT = "RS" # RS [axis]?
    MOVEREL = "R"
    MOVEABS = "M"
    HOME = "!"
    HERE = "H"  # [axis]=0 [axis]=0
    WHERE = "W"  # [axis] [axis]
    BACKLASH = "B" # [axis]=0 [axis]=0
    ARRAY = "AR"
    AHOME = "AH"
    LOAD = "LD"
    CNTS = "CNTS" # [axis]?
    J = "J"  # Joystick
    JS = "JS"  # Joystick
    RBMODE = "RM"  # Ring buffer mode setup.
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
    INFO = "INFO" # INFO [axis]


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


class FirmwareModules(Enum):
    SCAN_MODULE = "SCAN MODULE"
    ARRAY_MODULE = "ARRAY MODULE"


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
    START = 'S'
    STOP = 'P'
    # More read-only scan states exist, but are not captured here.


class ScanPattern(Enum):
    """parameter for specifying scan pattern."""
    RASTER = 0
    SERPENTINE = 1


class MicroMirrorControlMode(Enum):
    INTERNAL_INPUT = "0"
    EXTERNAL_INPUT = "1"

class PiezoControlMode(Enum):
    INTERNAL_CLOSED_LOOP = "0"
    EXTERNAL_CLOSED_LOOP = "1"
    INTERNAL_OPEN_LOOP = "2"
    EXTERNAL_OPEN_LOOP = "3"
    FAST = "+"
    SLOW = "-"


class TunableLensControlMode(Enum):
    TG1000_INPUT_NO_TEMP_COMPENSATION = "0"
    EXTERNAL_INPUT_NO_TEMP_COMPENSATION = "1"
    TG1000_INPUT_WITH_TEMP_COMPENSATION = "2"


class JoystickPolarity(Enum):
    # enum values are used in calculations elsewhere. Do not change them.
    INVERTED = 0
    DEFAULT = 1


class CCAZ(Enum):
    """Z parameter options for CCA command."""
    # more commands.
    AXIS_1_REVERSE_JOYSTICK_POLARITY = 22
    AXIS_1_RESET_POLARITY = 23
    AXIS_2_REVERSE_JOYSTICK_POLARITY = 24
    AXIS_2_RESET_JOYSTICK_POLARITY = 25
    # more commands.


class RingBufferMode(Enum):
    TTL = 0
    ONE_SHOT = 1
    REPEATING = 2


class TTLIn0Mode(Enum):
    OFF = 0
    MOVE_TO_NEXT_ABS_POSITION = 1
    REPEAT_LAST_REL_MOVE = 2
    AUTOFOCUS = 3
    ZSTACK_ENABLE = 4
    POSITION_REPORTING = 5  # Enabling this will probs break the driver.
    INTERRUPT_ENABLED = 6
    ARRAY_MODE_MOVE_TO_NEXT_POSITION = 7
    IN0_LOCK_TOGGLE = 9
    OUT0_TOGGLE_STATE = 10
    SERVOLOCK_MODE = 11
    MOVE_TO_NEXT_REL_POSITION = 12
    # more niche commands not included.
    SINGLE_AXIS_FUNCTION = 30


class TTLOut0Mode(Enum):
    ALWAYS_LOW = 0
    ALWAYS_HIGH = 1
    PULSE_AFTER_MOVING = 2
