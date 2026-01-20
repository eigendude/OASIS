################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS imu packet definitions."""


class ImuPacket:
    """
    Synchronized IMU packet data for AHRS updates.

    Contains imu_raw and imu_calibration measurements with a shared timestamp.
    The packet is validated for frame agreement and is used to apply gyro and
    accelerometer updates in a fixed order.
    """

    pass
