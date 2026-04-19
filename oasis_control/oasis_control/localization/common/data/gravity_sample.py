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
class GravitySample:
    """
    Canonical validated gravity-direction packet shared by AHRS components.

    Fields:
        timestamp_ns: measurement timestamp in integer nanoseconds
        frame_id: source gravity frame, expected to be imu_link
        gravity_mps2: measured physical gravity vector in the source frame
            that points down and is near 9.81 m/s^2 at rest
        gravity_covariance_mps2_2: 3x3 covariance in the source frame when
            available
    """

    timestamp_ns: int
    frame_id: str
    gravity_mps2: Vector3
    gravity_covariance_mps2_2: Optional[Matrix3]
