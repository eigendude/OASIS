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
from typing import Iterable
from typing import Optional

from oasis_control.localization.common.data.imu_sample import ImuSample
from oasis_control.localization.common.validation.imu_validation import (
    validate_imu_sample,
)


@dataclass(frozen=True)
class ImuGravityValidationResult:
    """
    Result of validating one incoming imu_gravity packet.

    Fields:
        accepted: true when the sample passed validation
        sample: validated canonical full IMU sample when accepted
        rejection_reason: short machine-facing reason for rejection
    """

    accepted: bool
    sample: Optional[ImuSample]
    rejection_reason: str


def validate_imu_gravity_sample(
    *,
    timestamp_ns: int,
    frame_id: str,
    expected_frame_id: str,
    orientation_xyzw: Iterable[float],
    orientation_covariance_row_major: Iterable[float],
    angular_velocity_rads: Iterable[float],
    angular_velocity_covariance_row_major: Iterable[float],
    linear_acceleration_mps2: Iterable[float],
    linear_acceleration_covariance_row_major: Iterable[float],
) -> ImuGravityValidationResult:
    """
    Validate an imu_gravity full-IMU sample against the AHRS contract
    """

    validation_result = validate_imu_sample(
        timestamp_ns=timestamp_ns,
        frame_id=frame_id,
        expected_frame_id=expected_frame_id,
        orientation_xyzw=orientation_xyzw,
        orientation_covariance_row_major=orientation_covariance_row_major,
        angular_velocity_rads=angular_velocity_rads,
        angular_velocity_covariance_row_major=angular_velocity_covariance_row_major,
        linear_acceleration_mps2=linear_acceleration_mps2,
        linear_acceleration_covariance_row_major=(
            linear_acceleration_covariance_row_major
        ),
    )
    return ImuGravityValidationResult(
        accepted=validation_result.accepted,
        sample=validation_result.sample,
        rejection_reason=validation_result.rejection_reason,
    )
