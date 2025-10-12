################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import enum


class AnalogMode(enum.Enum):
    DISABLED = enum.auto()
    INPUT = enum.auto()


class DigitalMode(enum.Enum):
    DISABLED = enum.auto()
    INPUT = enum.auto()
    INPUT_PULLUP = enum.auto()
    OUTPUT = enum.auto()
    PWM = enum.auto()
    SERVO = enum.auto()
    CPU_FAN_PWM = enum.auto()
    CPU_FAN_TACHOMETER = enum.auto()
