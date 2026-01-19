################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Quaternion helpers for the AHRS core math layer

Conventions:
    * Quaternions are stored in wxyz order
    * The nominal update uses q_new = q_nominal ⊗ Exp(delta_theta)
    * delta_theta is a rotation vector with magnitude equal to the rotation
      angle in radians
    * The small-angle branch uses the series expansion and normalizes the
      result for numerical stability
"""

from __future__ import annotations

import math


def quat_mul_wxyz(q_left: list[float], q_right: list[float]) -> list[float]:
    """
    Multiply two quaternions in wxyz order
    """

    w1: float = q_left[0]
    x1: float = q_left[1]
    y1: float = q_left[2]
    z1: float = q_left[3]

    w2: float = q_right[0]
    x2: float = q_right[1]
    y2: float = q_right[2]
    z2: float = q_right[3]

    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def quat_normalize_wxyz(q_wxyz: list[float]) -> list[float]:
    """
    Normalize a quaternion in wxyz order
    """

    norm: float = math.sqrt(
        q_wxyz[0] * q_wxyz[0]
        + q_wxyz[1] * q_wxyz[1]
        + q_wxyz[2] * q_wxyz[2]
        + q_wxyz[3] * q_wxyz[3]
    )
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be positive")
    inv: float = 1.0 / norm
    return [
        q_wxyz[0] * inv,
        q_wxyz[1] * inv,
        q_wxyz[2] * inv,
        q_wxyz[3] * inv,
    ]


def quat_from_rotvec_wxyz(rotvec: list[float]) -> list[float]:
    """
    Build a quaternion from a rotation vector using Exp(delta_theta)
    """

    rx: float = rotvec[0]
    ry: float = rotvec[1]
    rz: float = rotvec[2]
    angle: float = math.sqrt(rx * rx + ry * ry + rz * rz)

    # Rotation magnitude threshold in rad for small-angle series
    small_angle_rad: float = 1.0e-12

    if angle < small_angle_rad:
        dq_wb: list[float] = [1.0, 0.5 * rx, 0.5 * ry, 0.5 * rz]
        return quat_normalize_wxyz(dq_wb)

    # Half-angle term used in quaternion exponential
    half_angle: float = 0.5 * angle
    sin_half: float = math.sin(half_angle)
    inv_angle: float = 1.0 / angle
    return [
        math.cos(half_angle),
        rx * inv_angle * sin_half,
        ry * inv_angle * sin_half,
        rz * inv_angle * sin_half,
    ]
