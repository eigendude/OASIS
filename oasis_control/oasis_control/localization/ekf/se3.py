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
SE(3) math helpers for the EKF state
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3


_EPS: float = 1.0e-12


def quat_normalize(q_wxyz: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion in [w, x, y, z] order
    """

    norm: float = float(np.linalg.norm(q_wxyz))
    if norm < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q_wxyz / norm


def quat_multiply(q1_wxyz: np.ndarray, q2_wxyz: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions in [w, x, y, z] order
    """

    w1: float = float(q1_wxyz[0])
    x1: float = float(q1_wxyz[1])
    y1: float = float(q1_wxyz[2])
    z1: float = float(q1_wxyz[3])

    w2: float = float(q2_wxyz[0])
    x2: float = float(q2_wxyz[1])
    y2: float = float(q2_wxyz[2])
    z2: float = float(q2_wxyz[3])

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_conjugate(q_wxyz: np.ndarray) -> np.ndarray:
    """
    Conjugate a quaternion in [w, x, y, z] order
    """

    return np.array(
        [float(q_wxyz[0]), -float(q_wxyz[1]), -float(q_wxyz[2]), -float(q_wxyz[3])],
        dtype=float,
    )


def so3_exp(phi_rad: np.ndarray) -> np.ndarray:
    """
    Exponential map from so(3) to quaternion
    """

    angle_rad: float = float(np.linalg.norm(phi_rad))
    if angle_rad < _EPS:
        return quat_normalize(
            np.array(
                [
                    1.0,
                    0.5 * float(phi_rad[0]),
                    0.5 * float(phi_rad[1]),
                    0.5 * float(phi_rad[2]),
                ],
                dtype=float,
            )
        )

    axis: np.ndarray = phi_rad / angle_rad
    half_angle: float = 0.5 * angle_rad
    sin_half: float = math.sin(half_angle)
    return quat_normalize(
        np.array(
            [math.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half],
            dtype=float,
        )
    )


def so3_log(q_wxyz: np.ndarray) -> np.ndarray:
    """
    Log map from quaternion to so(3)
    """

    q_unit: np.ndarray = quat_normalize(q_wxyz)
    if q_unit[0] < 0.0:
        q_unit = -q_unit

    vector: np.ndarray = q_unit[1:4]
    sin_half: float = float(np.linalg.norm(vector))
    if sin_half < _EPS:
        return 2.0 * vector

    half_angle: float = math.atan2(sin_half, float(q_unit[0]))
    axis: np.ndarray = vector / sin_half
    return axis * (2.0 * half_angle)


def quat_to_rotmat(q_wxyz: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix
    """

    q_unit: np.ndarray = quat_normalize(q_wxyz)
    w: float = float(q_unit[0])
    x: float = float(q_unit[1])
    y: float = float(q_unit[2])
    z: float = float(q_unit[3])

    ww: float = w * w
    xx: float = x * x
    yy: float = y * y
    zz: float = z * z

    wx: float = w * x
    wy: float = w * y
    wz: float = w * z

    xy: float = x * y
    xz: float = x * z
    yz: float = y * z

    return np.array(
        [
            [ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz],
        ],
        dtype=float,
    )


def quat_to_rpy(q_wxyz: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion to roll, pitch, yaw in radians
    """

    q_unit: np.ndarray = quat_normalize(q_wxyz)
    w: float = float(q_unit[0])
    x: float = float(q_unit[1])
    y: float = float(q_unit[2])
    z: float = float(q_unit[3])

    sinr_cosp: float = 2.0 * (w * x + y * z)
    cosr_cosp: float = 1.0 - 2.0 * (x * x + y * y)
    roll: float = math.atan2(sinr_cosp, cosr_cosp)

    sinp: float = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch: float = math.copysign(0.5 * math.pi, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp: float = 2.0 * (w * z + x * y)
    cosy_cosp: float = 1.0 - 2.0 * (y * y + z * z)
    yaw: float = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def se3_adjoint(translation_m: np.ndarray, rotation_wxyz: np.ndarray) -> np.ndarray:
    """
    Compute the SE(3) adjoint matrix for a pose
    """

    rot: np.ndarray = quat_to_rotmat(rotation_wxyz)
    tx: float = float(translation_m[0])
    ty: float = float(translation_m[1])
    tz: float = float(translation_m[2])

    skew: np.ndarray = np.array(
        [[0.0, -tz, ty], [tz, 0.0, -tx], [-ty, tx, 0.0]], dtype=float
    )

    adjoint: np.ndarray = np.zeros((6, 6), dtype=float)
    adjoint[0:3, 0:3] = rot
    adjoint[0:3, 3:6] = skew @ rot
    adjoint[3:6, 3:6] = rot

    return adjoint


def pose_plus(
    translation_m: np.ndarray, rotation_wxyz: np.ndarray, delta_xi: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Apply a 6D tangent perturbation to a pose
    """

    delta_translation: np.ndarray = delta_xi[0:3]
    delta_rotation: np.ndarray = delta_xi[3:6]

    updated_translation: np.ndarray = translation_m + delta_translation
    delta_quat: np.ndarray = so3_exp(delta_rotation)
    updated_rotation: np.ndarray = quat_multiply(delta_quat, rotation_wxyz)

    return updated_translation, quat_normalize(updated_rotation)


def pose_minus(
    translation_a: np.ndarray,
    rotation_a_wxyz: np.ndarray,
    translation_b: np.ndarray,
    rotation_b_wxyz: np.ndarray,
) -> np.ndarray:
    """
    Compute the 6D tangent error from pose B to pose A
    """

    delta_translation: np.ndarray = translation_a - translation_b
    rot_delta: np.ndarray = quat_multiply(
        rotation_a_wxyz, quat_conjugate(rotation_b_wxyz)
    )
    delta_rotation: np.ndarray = so3_log(rot_delta)

    return np.hstack([delta_translation, delta_rotation]).astype(float)


def pose_to_ros_pose(translation_m: np.ndarray, rotation_wxyz: np.ndarray) -> Pose:
    """
    Convert a pose into a geometry_msgs/Pose
    """

    position: Point = Point(
        x=float(translation_m[0]),
        y=float(translation_m[1]),
        z=float(translation_m[2]),
    )
    orientation: Quaternion = Quaternion(
        w=float(rotation_wxyz[0]),
        x=float(rotation_wxyz[1]),
        y=float(rotation_wxyz[2]),
        z=float(rotation_wxyz[3]),
    )
    pose: Pose = Pose()
    pose.position = position
    pose.orientation = orientation
    return pose


def pose_to_ros_transform(
    translation_m: np.ndarray, rotation_wxyz: np.ndarray
) -> Transform:
    """
    Convert a pose into a geometry_msgs/Transform
    """

    translation: Vector3 = Vector3(
        x=float(translation_m[0]),
        y=float(translation_m[1]),
        z=float(translation_m[2]),
    )
    rotation: Quaternion = Quaternion(
        w=float(rotation_wxyz[0]),
        x=float(rotation_wxyz[1]),
        y=float(rotation_wxyz[2]),
        z=float(rotation_wxyz[3]),
    )
    transform: Transform = Transform()
    transform.translation = translation
    transform.rotation = rotation
    return transform
