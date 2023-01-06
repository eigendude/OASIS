################################################################################
#
#  Copyright (C) 2022 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class TelemetrixConstants:
    """
    This class contains a set of constants for Telemetrix use.
    """

    # Commands
    SET_MEMORY_REPORTING_INTERVAL: int = 55
    CPU_FAN_PWM_ATTACH: int = 56
    CPU_FAN_PWM_DETACH: int = 57
    CPU_FAN_TACH_ATTACH: int = 58
    CPU_FAN_TACH_DETACH: int = 59
    SET_CPU_FAN_SAMPLING_INTERVAL: int = 60
    CPU_FAN_WRITE: int = 61

    # Reports
    MEMORY_REPORT: int = 21
    CPU_FAN_TACH_REPORT: int = 22
