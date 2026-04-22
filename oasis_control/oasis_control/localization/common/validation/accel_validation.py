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

from oasis_control.localization.common.algebra.covariance import (
    parse_linear_covariance_3x3,
)
from oasis_control.localization.common.data.accel_sample import AccelSample
from oasis_control.localization.common.frames.frame_policy import frame_matches


@dataclass(frozen=True)
class AccelValidationResult:
    """
    Result of validating one incoming accel packet.

    Fields:
        accepted: true when the sample passed validation
        sample: validated canonical accel sample when accepted
        rejection_reason: short machine-facing reason for rejection
    """

    accepted: bool
    sample: Optional[AccelSample]
    rejection_reason: str


def validate_accel_sample(
    *,
    timestamp_ns: int,
    frame_id: str,
    expected_frame_id: str,
    accel_mps2: Iterable[float],
    accel_covariance_row_major: Iterable[float],
) -> AccelValidationResult:
    """
    Validate one incoming gravity-included accel sample against the AHRS contract
    """

    if not frame_matches(frame_id, expected_frame_id):
        return AccelValidationResult(False, None, "bad_frame")

    accel_vector_mps2 = _coerce_finite_vector3(accel_mps2)
    if accel_vector_mps2 is None:
        return AccelValidationResult(False, None, "bad_vector")

    accel_covariance_mps2_2 = parse_linear_covariance_3x3(accel_covariance_row_major)

    return AccelValidationResult(
        accepted=True,
        sample=AccelSample(
            timestamp_ns=int(timestamp_ns),
            frame_id=frame_id,
            accel_mps2=accel_vector_mps2,
            accel_covariance_mps2_2=accel_covariance_mps2_2,
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
