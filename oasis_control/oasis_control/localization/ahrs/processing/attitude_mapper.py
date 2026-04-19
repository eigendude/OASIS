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

from oasis_control.localization.common.data.imu_sample import ImuSample
from oasis_control.localization.common.frames.mounting import MountedImuSample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import apply_mounting_to_imu


def map_imu_to_base(
    imu_sample: ImuSample, mounting_transform: MountingTransform
) -> MountedImuSample:
    """
    Apply the fixed IMU mounting transform to one validated IMU sample.

    AHRS treats the driver-provided IMU covariance as the upstream sensor
    contract. When mounting is applied, the covariance is rotated into
    `base_link` without diagonalizing, tightening, or otherwise reinterpreting
    the full `3 x 3` structure.
    """

    return apply_mounting_to_imu(imu_sample, mounting_transform)
