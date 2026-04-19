################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from oasis_control.localization.common.algebra.quat import Matrix3
from oasis_control.localization.common.algebra.quat import Quaternion
from oasis_control.localization.common.algebra.quat import Vector3


@dataclass(frozen=True)
class ImuSample:
    """
    Canonical validated IMU packet shared by AHRS components.

    Fields:
        timestamp_ns: measurement timestamp in integer nanoseconds
        frame_id: source IMU frame, expected to be imu_link
        orientation_xyzw: canonical world-to-IMU quaternion `q_WI` in ROS
            xyzw order after driver-boundary normalization
        orientation_covariance_rad2: 3x3 orientation covariance in source frame
        orientation_covariance_unknown: true when ROS marks orientation
            covariance unknown with covariance[0] == -1
        angular_velocity_rads: angular velocity in the IMU frame
        angular_velocity_covariance_rads2: 3x3 angular covariance in the IMU
            frame when available
        linear_acceleration_mps2: linear acceleration in the IMU frame
        linear_acceleration_covariance_mps2_2: 3x3 linear acceleration
            covariance in the IMU frame when available
    """

    timestamp_ns: int
    frame_id: str
    orientation_xyzw: Quaternion
    orientation_covariance_rad2: Optional[Matrix3]
    orientation_covariance_unknown: bool
    angular_velocity_rads: Vector3
    angular_velocity_covariance_rads2: Optional[Matrix3]
    linear_acceleration_mps2: Vector3
    linear_acceleration_covariance_mps2_2: Optional[Matrix3]
