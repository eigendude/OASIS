################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import enum


class AnalogMode(enum.Enum):
    DISABLED: int = 0
    INPUT: int = 1


class DigitalMode(enum.Enum):
    DISABLED: int = 0
    INPUT: int = 1
    INPUT_PULLUP: int = 2
    OUTPUT: int = 3
    PWM: int = 4
    SERVO: int = 5
    CPU_FAN_PWM: int = 6
    CPU_FAN_TACHOMETER: int = 7
