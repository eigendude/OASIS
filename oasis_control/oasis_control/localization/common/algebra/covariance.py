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

from oasis_control.localization.common.algebra.quat import Matrix3
from oasis_control.localization.common.algebra.quat import transpose_matrix


UNKNOWN_ORIENTATION_COVARIANCE: list[float] = [
    -1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]


def parse_row_major_matrix3(values: Iterable[float]) -> Optional[Matrix3]:
    """
    Convert a row-major 3x3 sequence into a matrix when all values are finite.
    """

    row_major_values: tuple[float, ...] = tuple(float(value) for value in values)
    if len(row_major_values) != 9:
        return None

    if not all(math.isfinite(value) for value in row_major_values):
        return None

    return (
        (
            row_major_values[0],
            row_major_values[1],
            row_major_values[2],
        ),
        (
            row_major_values[3],
            row_major_values[4],
            row_major_values[5],
        ),
        (
            row_major_values[6],
            row_major_values[7],
            row_major_values[8],
        ),
    )


def parse_linear_covariance_3x3(values: Iterable[float]) -> Optional[Matrix3]:
    """
    Extract the leading linear 3x3 block from a row-major 6x6 covariance array.
    """

    covariance_values: tuple[float, ...] = tuple(float(value) for value in values)
    if len(covariance_values) != 36:
        return None

    linear_covariance_values: tuple[float, ...] = (
        covariance_values[0],
        covariance_values[1],
        covariance_values[2],
        covariance_values[6],
        covariance_values[7],
        covariance_values[8],
        covariance_values[12],
        covariance_values[13],
        covariance_values[14],
    )
    return parse_row_major_matrix3(linear_covariance_values)


def flatten_matrix3_row_major(matrix: Matrix3) -> list[float]:
    """
    Convert a 3x3 matrix into a row-major ROS-compatible list.
    """

    return [
        matrix[0][0],
        matrix[0][1],
        matrix[0][2],
        matrix[1][0],
        matrix[1][1],
        matrix[1][2],
        matrix[2][0],
        matrix[2][1],
        matrix[2][2],
    ]


def embed_linear_covariance_3x3(matrix: Matrix3) -> list[float]:
    """
    Embed a 3x3 covariance into the leading linear block of a 6x6 ROS array.
    """

    covariance_values: list[float] = [0.0] * 36
    covariance_values[0] = matrix[0][0]
    covariance_values[1] = matrix[0][1]
    covariance_values[2] = matrix[0][2]
    covariance_values[6] = matrix[1][0]
    covariance_values[7] = matrix[1][1]
    covariance_values[8] = matrix[1][2]
    covariance_values[12] = matrix[2][0]
    covariance_values[13] = matrix[2][1]
    covariance_values[14] = matrix[2][2]
    return covariance_values


def rotate_covariance(rotation_matrix: Matrix3, covariance: Matrix3) -> Matrix3:
    """
    Rotate a full 3x3 covariance using R * Sigma * R^T.
    """

    rotated_times_covariance: Matrix3 = _matrix_multiply(rotation_matrix, covariance)
    return _matrix_multiply(rotated_times_covariance, transpose_matrix(rotation_matrix))


def is_positive_semidefinite_diagonal(covariance: Matrix3) -> bool:
    """
    Return True when the covariance diagonal is finite and nonnegative.
    """

    return (
        covariance[0][0] >= 0.0
        and covariance[1][1] >= 0.0
        and covariance[2][2] >= 0.0
        and math.isfinite(covariance[0][0])
        and math.isfinite(covariance[1][1])
        and math.isfinite(covariance[2][2])
    )


def _matrix_multiply(lhs_matrix: Matrix3, rhs_matrix: Matrix3) -> Matrix3:
    return (
        (
            _dot3(lhs_matrix[0], _column(rhs_matrix, 0)),
            _dot3(lhs_matrix[0], _column(rhs_matrix, 1)),
            _dot3(lhs_matrix[0], _column(rhs_matrix, 2)),
        ),
        (
            _dot3(lhs_matrix[1], _column(rhs_matrix, 0)),
            _dot3(lhs_matrix[1], _column(rhs_matrix, 1)),
            _dot3(lhs_matrix[1], _column(rhs_matrix, 2)),
        ),
        (
            _dot3(lhs_matrix[2], _column(rhs_matrix, 0)),
            _dot3(lhs_matrix[2], _column(rhs_matrix, 1)),
            _dot3(lhs_matrix[2], _column(rhs_matrix, 2)),
        ),
    )


def _column(matrix: Matrix3, index: int) -> tuple[float, float, float]:
    return (matrix[0][index], matrix[1][index], matrix[2][index])


def _dot3(
    lhs_vector: tuple[float, float, float],
    rhs_vector: tuple[float, float, float],
) -> float:
    return (
        lhs_vector[0] * rhs_vector[0]
        + lhs_vector[1] * rhs_vector[1]
        + lhs_vector[2] * rhs_vector[2]
    )
