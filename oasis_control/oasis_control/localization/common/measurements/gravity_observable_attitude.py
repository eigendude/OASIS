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
# gravity-observable roll/pitch covariance
MIN_GRAVITY_NORM_MPS2: float = 1.0e-3

# Units: m/s^2
# Meaning: reject roll/pitch Jacobians near the gravity pitch singularity where
# the lateral YZ gravity magnitude collapses toward zero
MIN_ROLL_PITCH_OBSERVABLE_NORM_MPS2: float = 1.0e-3


@dataclass(frozen=True)
class GravityObservableRollPitchCovariance:
    """
    Gravity-observable roll/pitch covariance.

    Fields:
        roll_variance_rad2: roll variance in rad^2 from propagating gravity
            covariance through the OASIS roll equation
        pitch_variance_rad2: pitch variance in rad^2 from propagating gravity
            covariance through the OASIS pitch equation
        roll_pitch_covariance_rad2: roll/pitch covariance term in rad^2 from
            the same linearized propagation

    Gravity does not observe yaw, so this result intentionally contains only
    roll/pitch covariance terms.
    """

    roll_variance_rad2: float
    pitch_variance_rad2: float
    roll_pitch_covariance_rad2: float


def gravity_covariance_to_roll_pitch_covariance_rad2(
    gravity_mps2: Iterable[float],
    gravity_covariance_mps2_2: Optional[Iterable[Iterable[float]]],
) -> Optional[GravityObservableRollPitchCovariance]:
    """
    Propagate gravity covariance into roll/pitch covariance.

    This uses the same OASIS gravity-to-attitude convention as the boot
    mounting solver:

        roll = atan2(-g_y, -g_z)
        pitch = atan2(g_x, sqrt(g_y^2 + g_z^2))

    The local roll/pitch covariance is then approximated by first-order
    propagation:

        Sigma_roll,pitch ~= J * Sigma_g * J^T

    where `J` is the 2x3 Jacobian of the roll/pitch mapping with respect to
    gravity in the active frame. Invalid covariance inputs or near-singular
    gravity configurations return `None` so callers can publish an explicit
    unknown-covariance result. Yaw is intentionally omitted because gravity
    does not observe heading.
    """

    if gravity_covariance_mps2_2 is None:
        return None

    gravity_values_mps2: tuple[float, ...] = tuple(
        float(value) for value in gravity_mps2
    )
    if len(gravity_values_mps2) != 3:
        return None

    gravity_x_mps2: float = gravity_values_mps2[0]
    gravity_y_mps2: float = gravity_values_mps2[1]
    gravity_z_mps2: float = gravity_values_mps2[2]

    gravity_norm_mps2: float = math.sqrt(
        gravity_x_mps2 * gravity_x_mps2
        + gravity_y_mps2 * gravity_y_mps2
        + gravity_z_mps2 * gravity_z_mps2
    )
    if gravity_norm_mps2 < MIN_GRAVITY_NORM_MPS2:
        return None

    lateral_gravity_norm_mps2: float = math.sqrt(
        gravity_y_mps2 * gravity_y_mps2 + gravity_z_mps2 * gravity_z_mps2
    )
    if lateral_gravity_norm_mps2 < MIN_ROLL_PITCH_OBSERVABLE_NORM_MPS2:
        return None

    try:
        covariance_rows_mps2_2: tuple[tuple[float, ...], ...] = tuple(
            tuple(float(value) for value in row) for row in gravity_covariance_mps2_2
        )
    except (TypeError, ValueError):
        return None

    if len(covariance_rows_mps2_2) != 3:
        return None

    covariance_matrix_mps2_2: list[list[float]] = []
    row_index: int
    row_values_mps2_2: tuple[float, ...]
    for row_index, row_values_mps2_2 in enumerate(covariance_rows_mps2_2):
        if len(row_values_mps2_2) != 3:
            return None

        covariance_row_mps2_2: list[float] = []
        column_index: int
        covariance_value_mps2_2: float
        for column_index, covariance_value_mps2_2 in enumerate(row_values_mps2_2):
            if not math.isfinite(covariance_value_mps2_2):
                return None

            if row_index == column_index and covariance_value_mps2_2 < 0.0:
                return None

            covariance_row_mps2_2.append(covariance_value_mps2_2)

        covariance_matrix_mps2_2.append(covariance_row_mps2_2)

    symmetric_covariance_mps2_2: tuple[tuple[float, float, float], ...] = (
        (
            covariance_matrix_mps2_2[0][0],
            0.5 * (covariance_matrix_mps2_2[0][1] + covariance_matrix_mps2_2[1][0]),
            0.5 * (covariance_matrix_mps2_2[0][2] + covariance_matrix_mps2_2[2][0]),
        ),
        (
            0.5 * (covariance_matrix_mps2_2[1][0] + covariance_matrix_mps2_2[0][1]),
            covariance_matrix_mps2_2[1][1],
            0.5 * (covariance_matrix_mps2_2[1][2] + covariance_matrix_mps2_2[2][1]),
        ),
        (
            0.5 * (covariance_matrix_mps2_2[2][0] + covariance_matrix_mps2_2[0][2]),
            0.5 * (covariance_matrix_mps2_2[2][1] + covariance_matrix_mps2_2[1][2]),
            covariance_matrix_mps2_2[2][2],
        ),
    )

    lateral_gravity_norm_mps2_2: float = (
        lateral_gravity_norm_mps2 * lateral_gravity_norm_mps2
    )
    gravity_norm_mps2_2: float = gravity_norm_mps2 * gravity_norm_mps2

    # Derivatives of:
    # roll = atan2(-g_y, -g_z)
    roll_jacobian: tuple[float, float, float] = (
        0.0,
        gravity_z_mps2 / lateral_gravity_norm_mps2_2,
        -gravity_y_mps2 / lateral_gravity_norm_mps2_2,
    )

    # Derivatives of:
    # pitch = atan2(g_x, sqrt(g_y^2 + g_z^2))
    pitch_jacobian: tuple[float, float, float] = (
        lateral_gravity_norm_mps2 / gravity_norm_mps2_2,
        -gravity_x_mps2
        * gravity_y_mps2
        / (lateral_gravity_norm_mps2 * gravity_norm_mps2_2),
        -gravity_x_mps2
        * gravity_z_mps2
        / (lateral_gravity_norm_mps2 * gravity_norm_mps2_2),
    )

    roll_variance_rad2: float = _quadratic_form_3x3(
        vector=roll_jacobian,
        matrix=symmetric_covariance_mps2_2,
    )
    pitch_variance_rad2: float = _quadratic_form_3x3(
        vector=pitch_jacobian,
        matrix=symmetric_covariance_mps2_2,
    )
    roll_pitch_covariance_rad2: float = _bilinear_form_3x3(
        left_vector=roll_jacobian,
        matrix=symmetric_covariance_mps2_2,
        right_vector=pitch_jacobian,
    )

    if (
        not math.isfinite(roll_variance_rad2)
        or not math.isfinite(pitch_variance_rad2)
        or not math.isfinite(roll_pitch_covariance_rad2)
        or roll_variance_rad2 < 0.0
        or pitch_variance_rad2 < 0.0
    ):
        return None

    return GravityObservableRollPitchCovariance(
        roll_variance_rad2=roll_variance_rad2,
        pitch_variance_rad2=pitch_variance_rad2,
        roll_pitch_covariance_rad2=roll_pitch_covariance_rad2,
    )


def _quadratic_form_3x3(
    *,
    vector: tuple[float, float, float],
    matrix: tuple[tuple[float, float, float], ...],
) -> float:
    return _bilinear_form_3x3(
        left_vector=vector,
        matrix=matrix,
        right_vector=vector,
    )


def _bilinear_form_3x3(
    *,
    left_vector: tuple[float, float, float],
    matrix: tuple[tuple[float, float, float], ...],
    right_vector: tuple[float, float, float],
) -> float:
    left_x: float = left_vector[0]
    left_y: float = left_vector[1]
    left_z: float = left_vector[2]
    right_x: float = right_vector[0]
    right_y: float = right_vector[1]
    right_z: float = right_vector[2]

    return (
        left_x
        * (matrix[0][0] * right_x + matrix[0][1] * right_y + matrix[0][2] * right_z)
        + left_y
        * (matrix[1][0] * right_x + matrix[1][1] * right_y + matrix[1][2] * right_z)
        + left_z
        * (matrix[2][0] * right_x + matrix[2][1] * right_y + matrix[2][2] * right_z)
    )
