#!/usr/bin/env python3
"""TigerController Device Codes"""
from enum import Enum


class Cmds:
    # Common commands in bytes form.
    BUILD_X = b"BU X\r"
    STATUS = b"/\r"
    MOVEREL = b"R"
    MOVEABS = b"M"
    HOME = b"!"
    HERE = b"H"  # [axis]=0 [axis]=0
    WHERE = b"W"  # [axis] [axis]


class ErrorCodes(Enum):
    # Error message responses from the Tiger Controller
    UNKNOWN_CMD = ':N-1'
    UNRECOGNIZED_AXIS_PARAMETER = ':N-2'
    MISSING_PARAMETERS = ':N-3'
    PARAMETER_OUT_OF_RANGE = ':N-4'
    OPERATION_FAILED = ':N-5'
    UNDEFINED_ERROR = ':N-6'
    INVALID_CARD_ADDRESS = ':N-7'

    SERIAL_CMD_HALTED = ':N-21'

