################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS imu model definitions."""

class ImuModel:
    """
    IMU measurement model for gyro and accelerometer updates.

    Gyro measurement in frame {I}:

        z_ω = ω_raw
        ω̂_I = R_IB * ω_WB + b_g
        ν_ω = z_ω - ω̂_I

    Accelerometer measurement in frame {I}:

        z_a = a_raw
        f_B = R_WB * (a_WB - g_W)
        f_I = R_IB * f_B
        â_I = A_a⁻¹ * f_I + b_a
        ν_a = z_a - â_I

    All measurement covariances are full 3×3 matrices and are not diagonalized.
    """
    pass
