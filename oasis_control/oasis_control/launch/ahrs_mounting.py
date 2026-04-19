################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass


DEFAULT_AHRS_MOUNTING_PARENT_FRAME_ID: str = "base_link"
DEFAULT_AHRS_MOUNTING_CHILD_FRAME_ID: str = "imu_link"
DEFAULT_AHRS_MOUNTING_CALIBRATION_DURATION_SEC: float = 2.0
DEFAULT_AHRS_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS: float = 0.35
DEFAULT_AHRS_MOUNTING_MIN_SAMPLE_COUNT: int = 10


@dataclass(frozen=True)
class AhrsMountingConfig:
    """Launch-time boot calibration policy for the fixed AHRS mounting"""

    # Parent frame for the fixed mounting transform
    parent_frame_id: str = DEFAULT_AHRS_MOUNTING_PARENT_FRAME_ID

    # Child frame for the fixed mounting transform
    child_frame_id: str = DEFAULT_AHRS_MOUNTING_CHILD_FRAME_ID

    # Stationary boot window duration in seconds
    calibration_duration_sec: float = DEFAULT_AHRS_MOUNTING_CALIBRATION_DURATION_SEC

    # Maximum accepted angular speed during the stationary boot solve
    stationary_angular_speed_threshold_rads: float = (
        DEFAULT_AHRS_MOUNTING_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS
    )

    # Minimum accepted gravity samples before solving the fixed mounting
    min_sample_count: int = DEFAULT_AHRS_MOUNTING_MIN_SAMPLE_COUNT
