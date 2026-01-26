################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Type definitions for AHRS mounting calibration."""

from __future__ import annotations

from oasis_control.localization.mounting.mounting_types.diagnostics import Diagnostics
from oasis_control.localization.mounting.mounting_types.imu_packet import (
    ImuCalibrationPrior,
)
from oasis_control.localization.mounting.mounting_types.imu_packet import ImuPacket
from oasis_control.localization.mounting.mounting_types.keyframe import Keyframe
from oasis_control.localization.mounting.mounting_types.mag_packet import MagPacket
from oasis_control.localization.mounting.mounting_types.result_snapshot import SE3
from oasis_control.localization.mounting.mounting_types.result_snapshot import (
    ResultSnapshot,
)
from oasis_control.localization.mounting.mounting_types.steady_segment import (
    SteadySegment,
)
from oasis_control.localization.mounting.mounting_types.update_report import (
    UpdateReport,
)


__all__ = [
    "Diagnostics",
    "ImuCalibrationPrior",
    "ImuPacket",
    "Keyframe",
    "MagPacket",
    "ResultSnapshot",
    "SE3",
    "SteadySegment",
    "UpdateReport",
]
