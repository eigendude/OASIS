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

from oasis_control.localization.common.frames.mounting import MountedImuSample
from oasis_control.localization.common.measurements.gravity_direction import (
    GravityDirectionResidual,
)


@dataclass(frozen=True)
class AhrsOutput:
    """
    AHRS runtime output expressed in the mounted base frame.

    Fields:
        mounted_imu: mounted IMU signal block published on ahrs/imu
        gravity_residual: current gravity consistency summary when available
    """

    mounted_imu: MountedImuSample
    gravity_residual: Optional[GravityDirectionResidual]
