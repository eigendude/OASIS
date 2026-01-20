################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS ahrs state definitions."""


class AhrsState:
    """
    Mean AHRS state for navigation, calibration, and environment.

    State components:

    - p_WB: position of the body in world (meters)
    - v_WB: velocity of the body in world (m/s)
    - q_WB: unit quaternion rotating world → body
    - ω_WB: body angular rate in {B} (rad/s)
    - b_g: gyro bias in {I} (rad/s)
    - b_a: accel bias in {I} (m/s²)
    - A_a: accel scale/misalignment (unitless 3×3)
    - T_BI: IMU → body extrinsic (SE(3))
    - T_BM: mag → body extrinsic (SE(3))
    - g_W: gravity vector in world (m/s²)
    - m_W: magnetic field in world (tesla)
    """

    pass
