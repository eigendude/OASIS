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
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.frames.frame_policy import frame_matches


MIN_GRAVITY_VECTOR_NORM_MPS2: float = 1.0e-9


@dataclass(frozen=True)
class GravityValidationResult:
    """
    Result of validating one incoming gravity-direction packet.

    Fields:
        accepted: true when the sample passed validation
        sample: validated canonical gravity sample when accepted
        rejection_reason: short machine-facing reason for rejection
    """

    accepted: bool
    sample: Optional[GravitySample]
    rejection_reason: str


def validate_gravity_sample(
    *,
    timestamp_ns: int,
    frame_id: str,
    expected_frame_id: str,
    gravity_mps2: Iterable[float],
    gravity_covariance_row_major: Iterable[float],
) -> GravityValidationResult:
    """
    Validate one incoming gravity-direction sample against the AHRS contract.

    A finite nonzero gravity vector is accepted even when the covariance block
    is missing or unusable. In that case the sample stores no covariance and
    covariance-aware gating may remain unavailable downstream.
    """

    if not frame_matches(frame_id, expected_frame_id):
        return GravityValidationResult(False, None, "bad_frame")

    gravity_vector_mps2 = _coerce_finite_vector3(gravity_mps2)
    if gravity_vector_mps2 is None:
        return GravityValidationResult(False, None, "bad_vector")

    gravity_norm_mps2: float = math.sqrt(
        sum(component * component for component in gravity_vector_mps2)
    )
    if gravity_norm_mps2 <= MIN_GRAVITY_VECTOR_NORM_MPS2:
        return GravityValidationResult(False, None, "bad_vector")

    gravity_covariance_mps2_2 = parse_linear_covariance_3x3(
        gravity_covariance_row_major
    )

    return GravityValidationResult(
        accepted=True,
        sample=GravitySample(
            timestamp_ns=int(timestamp_ns),
            frame_id=frame_id,
            gravity_mps2=gravity_vector_mps2,
            gravity_covariance_mps2_2=gravity_covariance_mps2_2,
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
