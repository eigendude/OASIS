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
from oasis_control.localization.common.algebra.quat import Vector3


@dataclass(frozen=True)
class AccelSample:
    """
    Canonical validated accel packet for gravity-included acceleration.

    Fields:
        timestamp_ns: measurement timestamp in integer nanoseconds
        frame_id: source accel frame, expected to be imu_link
        accel_mps2: gravity-included acceleration in the IMU frame
        accel_covariance_mps2_2: 3x3 linear acceleration covariance in the IMU
            frame when available
    """

    timestamp_ns: int
    frame_id: str
    accel_mps2: Vector3
    accel_covariance_mps2_2: Optional[Matrix3]
