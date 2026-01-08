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
Quaternion and pose math helpers for EKF localization
"""

from __future__ import annotations

import math
from typing import Iterable
from typing import Sequence


# Units: unitless. Meaning: small-angle epsilon for quaternion log map
# derivation
_EPS_NORM: float = 1.0e-12


def normalize_quaternion(quat: Sequence[float]) -> tuple[float, float, float, float]:
    """
    Normalize a quaternion to unit length
    """

    x: float = float(quat[0])
    y: float = float(quat[1])
    z: float = float(quat[2])
    w: float = float(quat[3])
    norm: float = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv_norm: float = 1.0 / norm
    return (x * inv_norm, y * inv_norm, z * inv_norm, w * inv_norm)


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    """
    Multiply two quaternions in (x, y, z, w) order
    """

    lx: float = float(left[0])
    ly: float = float(left[1])
    lz: float = float(left[2])
    lw: float = float(left[3])
    rx: float = float(right[0])
    ry: float = float(right[1])
    rz: float = float(right[2])
    rw: float = float(right[3])

    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(quat: Sequence[float]) -> tuple[float, float, float, float]:
    """
    Compute the inverse of a quaternion
    """

    x: float = float(quat[0])
    y: float = float(quat[1])
    z: float = float(quat[2])
    w: float = float(quat[3])
    norm_sq: float = x * x + y * y + z * z + w * w
    if norm_sq <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv_norm_sq: float = 1.0 / norm_sq
    return (-x * inv_norm_sq, -y * inv_norm_sq, -z * inv_norm_sq, w * inv_norm_sq)


def quaternion_log(quat: Sequence[float]) -> tuple[float, float, float]:
    """
    Map a unit quaternion to its tangent-space rotation vector
    """

    x: float = float(quat[0])
    y: float = float(quat[1])
    z: float = float(quat[2])
    w: float = float(quat[3])
    v_norm: float = math.sqrt(x * x + y * y + z * z)
    if v_norm <= _EPS_NORM:
        return (0.0, 0.0, 0.0)

    angle: float = 2.0 * math.atan2(v_norm, w)
    scale: float = angle / v_norm
    return (x * scale, y * scale, z * scale)


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """
    Convert roll, pitch, yaw angles to a quaternion
    """

    half_roll: float = 0.5 * roll
    half_pitch: float = 0.5 * pitch
    half_yaw: float = 0.5 * yaw

    cr: float = math.cos(half_roll)
    sr: float = math.sin(half_roll)
    cp: float = math.cos(half_pitch)
    sp: float = math.sin(half_pitch)
    cy: float = math.cos(half_yaw)
    sy: float = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(
    quat: Sequence[float],
) -> tuple[float, float, float]:
    """
    Convert a quaternion to roll, pitch, yaw angles
    """

    x: float = float(quat[0])
    y: float = float(quat[1])
    z: float = float(quat[2])
    w: float = float(quat[3])

    sinr_cosp: float = 2.0 * (w * x + y * z)
    cosr_cosp: float = 1.0 - 2.0 * (x * x + y * y)
    roll: float = math.atan2(sinr_cosp, cosr_cosp)

    sinp: float = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch: float = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp: float = 2.0 * (w * z + x * y)
    cosy_cosp: float = 1.0 - 2.0 * (y * y + z * z)
    yaw: float = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def rotate_vector(
    quat: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    """
    Rotate a 3D vector by a quaternion
    """

    qx: float = float(quat[0])
    qy: float = float(quat[1])
    qz: float = float(quat[2])
    qw: float = float(quat[3])
    vx: float = float(vector[0])
    vy: float = float(vector[1])
    vz: float = float(vector[2])

    tx: float = 2.0 * (qy * vz - qz * vy)
    ty: float = 2.0 * (qz * vx - qx * vz)
    tz: float = 2.0 * (qx * vy - qy * vx)

    rx: float = vx + qw * tx + (qy * tz - qz * ty)
    ry: float = vy + qw * ty + (qz * tx - qx * tz)
    rz: float = vz + qw * tz + (qx * ty - qy * tx)

    return (rx, ry, rz)


def is_finite_vector(values: Iterable[float]) -> bool:
    """
    Check if all values in an iterable are finite
    """

    for value in values:
        if not math.isfinite(float(value)):
            return False
    return True
