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


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix3 = tuple[tuple[float, float, float], ...]


def normalize_quaternion_xyzw(
    quaternion_xyzw: Iterable[float],
) -> Optional[Quaternion]:
    """
    Return a unit quaternion in ROS xyzw order or None for invalid inputs.
    """

    values: tuple[float, ...] = tuple(float(value) for value in quaternion_xyzw)
    if len(values) != 4:
        return None

    if not all(math.isfinite(value) for value in values):
        return None

    norm: float = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        return None

    return (
        values[0] / norm,
        values[1] / norm,
        values[2] / norm,
        values[3] / norm,
    )


def quaternion_multiply_xyzw(
    lhs_quaternion_xyzw: Quaternion,
    rhs_quaternion_xyzw: Quaternion,
) -> Quaternion:
    """
    Compose two xyzw quaternions using the Hamilton product.
    """

    lhs_x, lhs_y, lhs_z, lhs_w = lhs_quaternion_xyzw
    rhs_x, rhs_y, rhs_z, rhs_w = rhs_quaternion_xyzw

    return (
        lhs_w * rhs_x + lhs_x * rhs_w + lhs_y * rhs_z - lhs_z * rhs_y,
        lhs_w * rhs_y - lhs_x * rhs_z + lhs_y * rhs_w + lhs_z * rhs_x,
        lhs_w * rhs_z + lhs_x * rhs_y - lhs_y * rhs_x + lhs_z * rhs_w,
        lhs_w * rhs_w - lhs_x * rhs_x - lhs_y * rhs_y - lhs_z * rhs_z,
    )


def quaternion_conjugate_xyzw(quaternion_xyzw: Quaternion) -> Quaternion:
    """
    Return the quaternion conjugate in ROS xyzw order.
    """

    return (
        -quaternion_xyzw[0],
        -quaternion_xyzw[1],
        -quaternion_xyzw[2],
        quaternion_xyzw[3],
    )


def quaternion_to_rotation_matrix(quaternion_xyzw: Quaternion) -> Matrix3:
    """
    Convert an xyzw quaternion into a 3x3 rotation matrix.
    """

    x_value, y_value, z_value, w_value = quaternion_xyzw

    xx_value: float = x_value * x_value
    xy_value: float = x_value * y_value
    xz_value: float = x_value * z_value
    xw_value: float = x_value * w_value
    yy_value: float = y_value * y_value
    yz_value: float = y_value * z_value
    yw_value: float = y_value * w_value
    zz_value: float = z_value * z_value
    zw_value: float = z_value * w_value

    return (
        (
            1.0 - 2.0 * (yy_value + zz_value),
            2.0 * (xy_value - zw_value),
            2.0 * (xz_value + yw_value),
        ),
        (
            2.0 * (xy_value + zw_value),
            1.0 - 2.0 * (xx_value + zz_value),
            2.0 * (yz_value - xw_value),
        ),
        (
            2.0 * (xz_value - yw_value),
            2.0 * (yz_value + xw_value),
            1.0 - 2.0 * (xx_value + yy_value),
        ),
    )


def rotate_vector(rotation_matrix: Matrix3, vector: Vector3) -> Vector3:
    """
    Apply a 3x3 rotation matrix to a 3D vector.
    """

    return (
        _dot(rotation_matrix[0], vector),
        _dot(rotation_matrix[1], vector),
        _dot(rotation_matrix[2], vector),
    )


def transpose_matrix(matrix: Matrix3) -> Matrix3:
    """
    Return the transpose of a 3x3 matrix.
    """

    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _dot(lhs_vector: Vector3, rhs_vector: Vector3) -> float:
    return (
        lhs_vector[0] * rhs_vector[0]
        + lhs_vector[1] * rhs_vector[1]
        + lhs_vector[2] * rhs_vector[2]
    )
