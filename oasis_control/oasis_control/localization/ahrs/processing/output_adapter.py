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

from typing import Optional

from oasis_control.localization.ahrs.data.ahrs_output import AhrsOutput
from oasis_control.localization.common.frames.mounting import MountedImuSample
from oasis_control.localization.common.measurements.gravity_direction import (
    GravityDirectionResidual,
)


def make_ahrs_output(
    mounted_imu_sample: MountedImuSample,
    gravity_residual: Optional[GravityDirectionResidual],
) -> AhrsOutput:
    """
    Bundle mounted IMU data with the current gravity consistency summary.
    """

    return AhrsOutput(
        mounted_imu=mounted_imu_sample,
        gravity_residual=gravity_residual,
    )
