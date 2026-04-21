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


# Units: m/s^2
# Meaning: reject degenerate gravity vectors when mapping gravity covariance to
# gravity-observable roll/pitch variance
MIN_GRAVITY_NORM_MPS2: float = 1.0e-3


@dataclass(frozen=True)
class GravityObservableAttitudeVariance:
    """
    Gravity-observable roll/pitch attitude variance.

    Fields:
        roll_variance_rad2: conservative roll variance in rad^2 derived from
            the measured gravity covariance and gravity magnitude
        pitch_variance_rad2: conservative pitch variance in rad^2 derived from
            the measured gravity covariance and gravity magnitude

    Gravity does not observe yaw, so this result intentionally contains only
    roll and pitch variance.
    """

    roll_variance_rad2: float
    pitch_variance_rad2: float


def gravity_covariance_to_roll_pitch_variance_rad2(
    gravity_mps2: Iterable[float],
    gravity_covariance_mps2_2: Optional[Iterable[Iterable[float]]],
) -> GravityObservableAttitudeVariance:
    """
    Approximate gravity-observable roll/pitch variance from gravity
    covariance.

    The runtime contribution uses the largest gravity linear covariance term as
    a conservative shared direction-noise scale:

        sigma_roll,pitch^2 ~= sigma_gravity^2 / |g|^2

    Invalid or unusable covariance inputs fall back to zero so callers can
    choose an explicit unknown-covariance or floor policy. The returned roll
    and pitch variances are identical under this conservative approximation.
    Yaw is intentionally omitted because gravity does not observe heading.
    """

    if gravity_covariance_mps2_2 is None:
        return GravityObservableAttitudeVariance(
            roll_variance_rad2=0.0,
            pitch_variance_rad2=0.0,
        )

    gravity_values_mps2: tuple[float, ...] = tuple(
        float(value) for value in gravity_mps2
    )
    if len(gravity_values_mps2) != 3:
        return GravityObservableAttitudeVariance(
            roll_variance_rad2=0.0,
            pitch_variance_rad2=0.0,
        )

    gravity_norm_mps2: float = math.sqrt(
        sum(value * value for value in gravity_values_mps2)
    )
    if gravity_norm_mps2 < MIN_GRAVITY_NORM_MPS2:
        return GravityObservableAttitudeVariance(
            roll_variance_rad2=0.0,
            pitch_variance_rad2=0.0,
        )

    try:
        covariance_rows_mps2_2: tuple[tuple[float, ...], ...] = tuple(
            tuple(float(value) for value in row) for row in gravity_covariance_mps2_2
        )
    except (TypeError, ValueError):
        return GravityObservableAttitudeVariance(
            roll_variance_rad2=0.0,
            pitch_variance_rad2=0.0,
        )

    if len(covariance_rows_mps2_2) != 3:
        return GravityObservableAttitudeVariance(
            roll_variance_rad2=0.0,
            pitch_variance_rad2=0.0,
        )

    diagonal_variances_mps2_2: list[float] = []
    row_index: int
    row_values_mps2_2: tuple[float, ...]
    for row_index, row_values_mps2_2 in enumerate(covariance_rows_mps2_2):
        if len(row_values_mps2_2) != 3:
            return GravityObservableAttitudeVariance(
                roll_variance_rad2=0.0,
                pitch_variance_rad2=0.0,
            )

        diagonal_variance_mps2_2: float = row_values_mps2_2[row_index]
        if not math.isfinite(diagonal_variance_mps2_2):
            return GravityObservableAttitudeVariance(
                roll_variance_rad2=0.0,
                pitch_variance_rad2=0.0,
            )

        if diagonal_variance_mps2_2 < 0.0:
            return GravityObservableAttitudeVariance(
                roll_variance_rad2=0.0,
                pitch_variance_rad2=0.0,
            )

        diagonal_variances_mps2_2.append(diagonal_variance_mps2_2)

    shared_variance_rad2: float = max(diagonal_variances_mps2_2) / (
        gravity_norm_mps2 * gravity_norm_mps2
    )
    return GravityObservableAttitudeVariance(
        roll_variance_rad2=shared_variance_rad2,
        pitch_variance_rad2=shared_variance_rad2,
    )
