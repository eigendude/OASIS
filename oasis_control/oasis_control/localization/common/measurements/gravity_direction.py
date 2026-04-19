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
from typing import Optional

from oasis_control.localization.common.algebra.quat import Matrix3
from oasis_control.localization.common.algebra.quat import Quaternion
from oasis_control.localization.common.algebra.quat import Vector3
from oasis_control.localization.common.algebra.quat import quaternion_to_rotation_matrix
from oasis_control.localization.common.algebra.quat import rotate_vector


WORLD_GRAVITY_DIRECTION: Vector3 = (0.0, 0.0, -1.0)
MIN_DIRECTION_NORM: float = 1.0e-9

# Units: direction^2
# Meaning: tiny covariance floor added after normalization so the covariance
# remains numerically invertible for Mahalanobis gating in the tangent space
NORMALIZED_COVARIANCE_FLOOR: float = 1.0e-12


@dataclass(frozen=True)
class GravityDirectionResidual:
    """
    Gravity consistency summary expressed in the mounted base frame.

    Fields:
        measured_direction: measured unit gravity direction in base coordinates
        predicted_direction: predicted unit gravity direction from attitude
        residual_vector: measured minus predicted unit-direction residual
        residual_norm: Euclidean norm of residual_vector
        mahalanobis_distance: covariance-aware residual distance when usable
    """

    measured_direction: Vector3
    predicted_direction: Vector3
    residual_vector: Vector3
    residual_norm: float
    mahalanobis_distance: float


def compute_gravity_direction_residual(
    *,
    measured_gravity_mps2: Vector3,
    measured_gravity_covariance_mps2_2: Optional[Matrix3],
    mounted_orientation_xyzw: Quaternion,
) -> GravityDirectionResidual:
    """
    Compare measured mounted gravity against the attitude-predicted direction.
    """

    measured_direction, measured_norm_mps2 = _normalize_vector(measured_gravity_mps2)
    rotation_world_to_base: Matrix3 = quaternion_to_rotation_matrix(
        mounted_orientation_xyzw
    )
    predicted_direction: Vector3 = rotate_vector(
        rotation_world_to_base, WORLD_GRAVITY_DIRECTION
    )
    residual_vector: Vector3 = (
        measured_direction[0] - predicted_direction[0],
        measured_direction[1] - predicted_direction[1],
        measured_direction[2] - predicted_direction[2],
    )
    residual_norm: float = math.sqrt(
        residual_vector[0] * residual_vector[0]
        + residual_vector[1] * residual_vector[1]
        + residual_vector[2] * residual_vector[2]
    )

    mahalanobis_distance: float = math.nan
    if residual_norm <= 0.0:
        mahalanobis_distance = 0.0

    if measured_gravity_covariance_mps2_2 is not None:
        normalized_covariance: Optional[Matrix3] = _normalize_covariance(
            measured_direction=measured_direction,
            measured_norm_mps2=measured_norm_mps2,
            measured_covariance_mps2_2=measured_gravity_covariance_mps2_2,
        )
        if normalized_covariance is not None:
            inverse_covariance: Optional[Matrix3] = _invert_symmetric_matrix3(
                normalized_covariance
            )
            if inverse_covariance is not None:
                quadratic_form: float = _quadratic_form(
                    inverse_covariance, residual_vector
                )
                if quadratic_form >= 0.0:
                    mahalanobis_distance = math.sqrt(quadratic_form)

    return GravityDirectionResidual(
        measured_direction=measured_direction,
        predicted_direction=predicted_direction,
        residual_vector=residual_vector,
        residual_norm=residual_norm,
        mahalanobis_distance=mahalanobis_distance,
    )


def _normalize_vector(vector: Vector3) -> tuple[Vector3, float]:
    norm: float = math.sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]
    )
    if norm <= MIN_DIRECTION_NORM:
        raise ValueError("gravity vector norm must be positive")

    return (
        (
            vector[0] / norm,
            vector[1] / norm,
            vector[2] / norm,
        ),
        norm,
    )


def _normalize_covariance(
    *,
    measured_direction: Vector3,
    measured_norm_mps2: float,
    measured_covariance_mps2_2: Matrix3,
) -> Optional[Matrix3]:
    # Units: 1 / (m/s^2)
    # Meaning: Jacobian of vector normalization u = g / |g|
    # Derivation: du/dg = (I - u u^T) / |g|
    jacobian: Matrix3 = (
        (
            (1.0 - measured_direction[0] * measured_direction[0]) / measured_norm_mps2,
            (-measured_direction[0] * measured_direction[1]) / measured_norm_mps2,
            (-measured_direction[0] * measured_direction[2]) / measured_norm_mps2,
        ),
        (
            (-measured_direction[1] * measured_direction[0]) / measured_norm_mps2,
            (1.0 - measured_direction[1] * measured_direction[1]) / measured_norm_mps2,
            (-measured_direction[1] * measured_direction[2]) / measured_norm_mps2,
        ),
        (
            (-measured_direction[2] * measured_direction[0]) / measured_norm_mps2,
            (-measured_direction[2] * measured_direction[1]) / measured_norm_mps2,
            (1.0 - measured_direction[2] * measured_direction[2]) / measured_norm_mps2,
        ),
    )
    normalized_covariance: Matrix3 = _matrix_multiply(
        _matrix_multiply(jacobian, measured_covariance_mps2_2),
        _transpose(jacobian),
    )
    normalized_covariance = (
        (
            normalized_covariance[0][0] + NORMALIZED_COVARIANCE_FLOOR,
            normalized_covariance[0][1],
            normalized_covariance[0][2],
        ),
        (
            normalized_covariance[1][0],
            normalized_covariance[1][1] + NORMALIZED_COVARIANCE_FLOOR,
            normalized_covariance[1][2],
        ),
        (
            normalized_covariance[2][0],
            normalized_covariance[2][1],
            normalized_covariance[2][2] + NORMALIZED_COVARIANCE_FLOOR,
        ),
    )

    if not all(
        math.isfinite(component) for row in normalized_covariance for component in row
    ):
        return None

    return normalized_covariance


def _invert_symmetric_matrix3(matrix: Matrix3) -> Optional[Matrix3]:
    determinant: float = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    if not math.isfinite(determinant) or abs(determinant) <= 1.0e-30:
        return None

    inverse_scale: float = 1.0 / determinant
    inverse_matrix: Matrix3 = (
        (
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * inverse_scale,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * inverse_scale,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * inverse_scale,
        ),
        (
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * inverse_scale,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * inverse_scale,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * inverse_scale,
        ),
        (
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * inverse_scale,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * inverse_scale,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * inverse_scale,
        ),
    )

    if not all(math.isfinite(component) for row in inverse_matrix for component in row):
        return None

    return inverse_matrix


def _quadratic_form(matrix: Matrix3, vector: Vector3) -> float:
    return (
        vector[0]
        * (
            matrix[0][0] * vector[0]
            + matrix[0][1] * vector[1]
            + matrix[0][2] * vector[2]
        )
        + vector[1]
        * (
            matrix[1][0] * vector[0]
            + matrix[1][1] * vector[1]
            + matrix[1][2] * vector[2]
        )
        + vector[2]
        * (
            matrix[2][0] * vector[0]
            + matrix[2][1] * vector[1]
            + matrix[2][2] * vector[2]
        )
    )


def _matrix_multiply(lhs_matrix: Matrix3, rhs_matrix: Matrix3) -> Matrix3:
    return (
        (
            _dot(lhs_matrix[0], _column(rhs_matrix, 0)),
            _dot(lhs_matrix[0], _column(rhs_matrix, 1)),
            _dot(lhs_matrix[0], _column(rhs_matrix, 2)),
        ),
        (
            _dot(lhs_matrix[1], _column(rhs_matrix, 0)),
            _dot(lhs_matrix[1], _column(rhs_matrix, 1)),
            _dot(lhs_matrix[1], _column(rhs_matrix, 2)),
        ),
        (
            _dot(lhs_matrix[2], _column(rhs_matrix, 0)),
            _dot(lhs_matrix[2], _column(rhs_matrix, 1)),
            _dot(lhs_matrix[2], _column(rhs_matrix, 2)),
        ),
    )


def _transpose(matrix: Matrix3) -> Matrix3:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _column(matrix: Matrix3, index: int) -> Vector3:
    return (matrix[0][index], matrix[1][index], matrix[2][index])


def _dot(lhs_vector: Vector3, rhs_vector: Vector3) -> float:
    return (
        lhs_vector[0] * rhs_vector[0]
        + lhs_vector[1] * rhs_vector[1]
        + lhs_vector[2] * rhs_vector[2]
    )
