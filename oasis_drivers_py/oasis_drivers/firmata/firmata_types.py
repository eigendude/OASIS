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
    DISABLED = 0
    INPUT = 1


class DigitalMode(enum.Enum):
    DISABLED = 0
    INPUT = 1
    INPUT_PULLUP = 2
    OUTPUT = 3
    PWM = 4
    SERVO = 5
