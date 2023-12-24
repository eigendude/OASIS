################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
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
    I2C_CCS811_BEGIN: int = 62
    I2C_CCS811_END: int = 63
    I2C_MPU6050_BEGIN: int = 64
    I2C_MPU6050_END: int = 65
    ENABLE_LOGGING: int = 66
    DISABLE_LOGGING: int = 67

    # Reports
    MEMORY_REPORT: int = 21
    CPU_FAN_TACH_REPORT: int = 22
    AQ_CO2_TVOC_REPORT: int = 23
    IMU_6_AXIS_REPORT: int = 24
    STRING_DATA: int = 25
