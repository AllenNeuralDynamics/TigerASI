#!/usr/bin/env python3
"""TigerController Device Codes"""
from enum import Enum


class Cmds:
    # Common commands in bytes form.
    BUILD_X = b"BU X\r"
    HALT = b"\\\r"
    STATUS = b"/\r"
    RDSTAT = b"RS" # RS [axis]?
    MOVEREL = b"R"
    MOVEABS = b"M"
    HOME = b"!"
    HERE = b"H"  # [axis]=0 [axis]=0
    WHERE = b"W"  # [axis] [axis]
    BACKLASH = b"B" # [axis]=0 [axis]=0
    CNTS = b"CNTS" # [axis]?
    SCAN = b"SCAN" # [X?] [Y=fast_axis_id, default X] [Z=slow_axis_id, default Y] [F=pattern]
    SCANR = b"SCANR" # fast_axis [X=start in mm] [Y=stop in mm] [Z=enc_divide] [F= #_pixels] [R=retrace_speed]
    SCANV = b"SCANV" # slow_axis [X=start in mm] [Y=stop in mm] [Z=number_of_lines] [F=overshoot_time in ms] [T=scan_overshoot]
    SETHOME = b"HM"
    SETLOW = b"SL"
    SETUP = b"SU"
    SPEED = b"S"
    TTL = b"TTL" # [X=IN0_mode] [Y=OUT0_mode] [Z=aux_IO_state] [F=OUT0_polarity] [R=aux_IO_mask] [T=aux_IO_mode]
    PM = b"PM" # PM [axis]=[0 or 1] for mirror, [0 or 3] for ETL
    PZINFO = b"PZINFO" # [card address]PZINFO


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

