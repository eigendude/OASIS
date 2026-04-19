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
from typing import Iterable
from typing import Optional


# Units: m/s^2
# Meaning: reject degenerate gravity vectors when mapping gravity covariance to
# a tilt variance scale
MIN_GRAVITY_NORM_MPS2: float = 1.0e-3


def gravity_covariance_to_tilt_variance_rad2(
    gravity_mps2: Iterable[float],
    gravity_covariance_mps2_2: Optional[Iterable[Iterable[float]]],
) -> float:
    """
    Approximate tilt variance from gravity covariance with a small-angle model.

    The runtime contribution uses the largest gravity linear covariance term as
    a conservative direction-noise scale:

        sigma_tilt^2 ~= sigma_gravity^2 / |g|^2

    Invalid or unusable covariance inputs fall back to zero so callers can
    choose an explicit unknown-covariance or floor policy.
    """

    if gravity_covariance_mps2_2 is None:
        return 0.0

    gravity_values_mps2: tuple[float, ...] = tuple(
        float(value) for value in gravity_mps2
    )
    if len(gravity_values_mps2) != 3:
        return 0.0

    gravity_norm_mps2: float = math.sqrt(
        sum(value * value for value in gravity_values_mps2)
    )
    if gravity_norm_mps2 < MIN_GRAVITY_NORM_MPS2:
        return 0.0

    try:
        covariance_rows_mps2_2: tuple[tuple[float, ...], ...] = tuple(
            tuple(float(value) for value in row) for row in gravity_covariance_mps2_2
        )
    except (TypeError, ValueError):
        return 0.0

    if len(covariance_rows_mps2_2) != 3:
        return 0.0

    diagonal_variances_mps2_2: list[float] = []
    row_index: int
    row_values_mps2_2: tuple[float, ...]
    for row_index, row_values_mps2_2 in enumerate(covariance_rows_mps2_2):
        if len(row_values_mps2_2) != 3:
            return 0.0

        diagonal_variance_mps2_2: float = row_values_mps2_2[row_index]
        if not math.isfinite(diagonal_variance_mps2_2):
            return 0.0

        if diagonal_variance_mps2_2 < 0.0:
            return 0.0

        diagonal_variances_mps2_2.append(diagonal_variance_mps2_2)

    sigma_gravity_mps2_2: float = max(diagonal_variances_mps2_2)
    return sigma_gravity_mps2_2 / (gravity_norm_mps2 * gravity_norm_mps2)
