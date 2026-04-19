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

import math
from dataclasses import dataclass
from typing import Iterable
from typing import Optional

from oasis_control.localization.common.algebra.covariance import parse_row_major_matrix3
from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.data.imu_sample import ImuSample
from oasis_control.localization.common.frames.frame_policy import frame_matches


@dataclass(frozen=True)
class ImuValidationResult:
    """
    Result of validating one incoming IMU packet.

    Fields:
        accepted: true when the sample passed validation
        sample: validated canonical IMU sample when accepted
        rejection_reason: short machine-facing reason for rejection
    """

    accepted: bool
    sample: Optional[ImuSample]
    rejection_reason: str


def validate_imu_sample(
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
) -> ImuValidationResult:
    """
    Validate one incoming IMU sample against the AHRS contract.
    """

    if not frame_matches(frame_id, expected_frame_id):
        return ImuValidationResult(False, None, "bad_frame")

    normalized_quaternion_xyzw = normalize_quaternion_xyzw(orientation_xyzw)
    if normalized_quaternion_xyzw is None:
        return ImuValidationResult(False, None, "bad_orientation")

    angular_velocity_vector_rads = _coerce_finite_vector3(angular_velocity_rads)
    if angular_velocity_vector_rads is None:
        return ImuValidationResult(False, None, "bad_angular_velocity")

    linear_acceleration_vector_mps2 = _coerce_finite_vector3(linear_acceleration_mps2)
    if linear_acceleration_vector_mps2 is None:
        return ImuValidationResult(False, None, "bad_linear_acceleration")

    orientation_covariance_values: tuple[float, ...] = tuple(
        float(value) for value in orientation_covariance_row_major
    )
    if len(orientation_covariance_values) != 9:
        return ImuValidationResult(False, None, "bad_orientation_covariance")

    orientation_covariance_unknown: bool = orientation_covariance_values[0] == -1.0
    orientation_covariance_rad2 = None
    if not orientation_covariance_unknown:
        orientation_covariance_rad2 = parse_row_major_matrix3(
            orientation_covariance_values
        )
        if orientation_covariance_rad2 is None:
            return ImuValidationResult(False, None, "bad_orientation_covariance")

    angular_velocity_covariance_rads2 = parse_row_major_matrix3(
        angular_velocity_covariance_row_major
    )
    if angular_velocity_covariance_rads2 is None:
        return ImuValidationResult(False, None, "bad_angular_covariance")

    linear_acceleration_covariance_mps2_2 = parse_row_major_matrix3(
        linear_acceleration_covariance_row_major
    )
    if linear_acceleration_covariance_mps2_2 is None:
        return ImuValidationResult(False, None, "bad_linear_covariance")

    return ImuValidationResult(
        accepted=True,
        sample=ImuSample(
            timestamp_ns=int(timestamp_ns),
            frame_id=frame_id,
            orientation_xyzw=normalized_quaternion_xyzw,
            orientation_covariance_rad2=orientation_covariance_rad2,
            orientation_covariance_unknown=orientation_covariance_unknown,
            angular_velocity_rads=angular_velocity_vector_rads,
            angular_velocity_covariance_rads2=angular_velocity_covariance_rads2,
            linear_acceleration_mps2=linear_acceleration_vector_mps2,
            linear_acceleration_covariance_mps2_2=(
                linear_acceleration_covariance_mps2_2
            ),
        ),
        rejection_reason="",
    )


def _coerce_finite_vector3(
    values: Iterable[float],
) -> Optional[tuple[float, float, float]]:
    vector_values: tuple[float, ...] = tuple(float(value) for value in values)
    if len(vector_values) != 3:
        return None

    if not all(math.isfinite(value) for value in vector_values):
        return None

    return (
        vector_values[0],
        vector_values[1],
        vector_values[2],
    )
