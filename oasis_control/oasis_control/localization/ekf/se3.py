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
SE(3) helper utilities for EKF state handling
"""

from __future__ import annotations

import math
from typing import Any
from typing import Optional
from typing import Sequence

import numpy as np


# Small-angle threshold for rotation vector magnitude in radians
_ROT_EPS: float = 1.0e-12


def normalize_quaternion(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion to unit length
    """

    quat: np.ndarray = np.asarray(quat_wxyz, dtype=float).reshape(4)
    norm: float = float(np.linalg.norm(quat))
    if norm <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm


def quat_multiply(lhs_wxyz: np.ndarray, rhs_wxyz: np.ndarray) -> np.ndarray:
    """
    Multiply quaternions in wxyz order
    """

    lhs: np.ndarray = np.asarray(lhs_wxyz, dtype=float).reshape(4)
    rhs: np.ndarray = np.asarray(rhs_wxyz, dtype=float).reshape(4)
    w1: float = float(lhs[0])
    x1: float = float(lhs[1])
    y1: float = float(lhs[2])
    z1: float = float(lhs[3])
    w2: float = float(rhs[0])
    x2: float = float(rhs[1])
    y2: float = float(rhs[2])
    z2: float = float(rhs[3])
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_conjugate(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Conjugate a quaternion in wxyz order
    """

    quat: np.ndarray = np.asarray(quat_wxyz, dtype=float).reshape(4)
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=float)


def quat_inverse(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Invert a quaternion in wxyz order
    """

    quat: np.ndarray = np.asarray(quat_wxyz, dtype=float).reshape(4)
    norm_sq: float = float(np.dot(quat, quat))
    if norm_sq <= 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat_conjugate(quat) / norm_sq


def quat_from_rotvec(rotvec: np.ndarray | Sequence[float]) -> np.ndarray:
    """
    Map a rotation vector into a quaternion in wxyz order. Accepts a
    length-3 sequence or ndarray.
    """

    phi: np.ndarray = np.asarray(rotvec, dtype=float).reshape(3)
    angle: float = float(np.linalg.norm(phi))
    if angle < _ROT_EPS:
        half: float = 0.5
        return normalize_quaternion(
            np.array([1.0, half * phi[0], half * phi[1], half * phi[2]], dtype=float)
        )
    axis: np.ndarray = phi / angle
    half_angle: float = 0.5 * angle
    sin_half: float = float(math.sin(half_angle))
    return normalize_quaternion(
        np.array(
            [
                math.cos(half_angle),
                axis[0] * sin_half,
                axis[1] * sin_half,
                axis[2] * sin_half,
            ],
            dtype=float,
        )
    )


def rotvec_from_quat(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Map a quaternion in wxyz order into a rotation vector
    """

    quat: np.ndarray = normalize_quaternion(quat_wxyz)
    if quat[0] < 0.0:
        quat = -quat
    vec: np.ndarray = quat[1:]
    vec_norm: float = float(np.linalg.norm(vec))
    if vec_norm < _ROT_EPS:
        return np.zeros(3, dtype=float)
    angle: float = 2.0 * math.atan2(vec_norm, float(quat[0]))
    axis: np.ndarray = vec / vec_norm
    return axis * angle


def quat_to_rotation_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Convert quaternion in wxyz order to a rotation matrix
    """

    quat: np.ndarray = normalize_quaternion(quat_wxyz)
    w: float = float(quat[0])
    x: float = float(quat[1])
    y: float = float(quat[2])
    z: float = float(quat[3])
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def quat_to_rpy(quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Convert quaternion in wxyz order to roll, pitch, yaw
    """

    quat: np.ndarray = normalize_quaternion(quat_wxyz)
    w: float = float(quat[0])
    x: float = float(quat[1])
    y: float = float(quat[2])
    z: float = float(quat[3])

    sin_roll: float = 2.0 * (w * x + y * z)
    cos_roll: float = 1.0 - 2.0 * (x * x + y * y)
    roll: float = math.atan2(sin_roll, cos_roll)

    sin_pitch: float = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch: float = math.asin(sin_pitch)

    sin_yaw: float = 2.0 * (w * z + x * y)
    cos_yaw: float = 1.0 - 2.0 * (y * y + z * z)
    yaw: float = math.atan2(sin_yaw, cos_yaw)

    return np.array([roll, pitch, yaw], dtype=float)


def se3_adjoint(translation_m: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """
    Compute the SE(3) adjoint for a pose
    """

    rot: np.ndarray = quat_to_rotation_matrix(quat_wxyz)
    skew: np.ndarray = _skew_symmetric(translation_m)
    adjoint: np.ndarray = np.zeros((6, 6), dtype=float)
    adjoint[:3, :3] = rot
    adjoint[:3, 3:] = skew @ rot
    adjoint[3:, 3:] = rot
    return adjoint


def pose_plus(
    translation_m: np.ndarray, quat_wxyz: np.ndarray, delta: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Apply a tangent-space perturbation to a pose
    """

    delta_vec: np.ndarray = np.asarray(delta, dtype=float).reshape(6)
    delta_t: np.ndarray = delta_vec[:3]
    delta_rot: np.ndarray = delta_vec[3:]
    new_translation: np.ndarray = np.asarray(translation_m, dtype=float).reshape(3)
    new_translation = new_translation + delta_t
    delta_quat: np.ndarray = quat_from_rotvec(delta_rot)
    new_quat: np.ndarray = quat_multiply(quat_wxyz, delta_quat)
    return new_translation, normalize_quaternion(new_quat)


def pose_compose(
    translation_a: np.ndarray,
    quat_a_wxyz: np.ndarray,
    translation_b: np.ndarray,
    quat_b_wxyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compose two poses with left-multiplication
    """

    rot_a: np.ndarray = quat_to_rotation_matrix(quat_a_wxyz)
    translated: np.ndarray = (
        np.asarray(translation_a, dtype=float).reshape(3)
        + rot_a @ np.asarray(translation_b, dtype=float).reshape(3)
    )
    quat: np.ndarray = quat_multiply(quat_a_wxyz, quat_b_wxyz)
    return translated, normalize_quaternion(quat)


def pose_minus(
    t1_m: np.ndarray,
    q1_wxyz: np.ndarray,
    t2_m: np.ndarray,
    q2_wxyz: np.ndarray,
) -> np.ndarray:
    """
    Compute a tangent-space delta between two poses
    """

    delta_t: np.ndarray = np.asarray(t1_m, dtype=float).reshape(3) - np.asarray(
        t2_m, dtype=float
    ).reshape(3)
    q_rel: np.ndarray = quat_multiply(q1_wxyz, quat_inverse(q2_wxyz))
    delta_rot: np.ndarray = rotvec_from_quat(q_rel)
    return np.concatenate((delta_t, delta_rot), axis=0)


def to_ros_pose(
    translation_m: np.ndarray, quat_wxyz: np.ndarray, pose_msg: Optional[Any] = None
) -> Any:
    """
    Populate a geometry_msgs/Pose message from translation and quaternion
    """

    try:
        from geometry_msgs.msg import Pose as PoseMsg
    except ModuleNotFoundError as exc:
        raise RuntimeError("geometry_msgs is required for to_ros_pose") from exc

    pose_msg = PoseMsg()
    pose_msg.position.x = float(translation_m[0])
    pose_msg.position.y = float(translation_m[1])
    pose_msg.position.z = float(translation_m[2])
    pose_msg.orientation.w = float(quat_wxyz[0])
    pose_msg.orientation.x = float(quat_wxyz[1])
    pose_msg.orientation.y = float(quat_wxyz[2])
    pose_msg.orientation.z = float(quat_wxyz[3])
    return pose_msg


def to_ros_transform(
    translation_m: np.ndarray,
    quat_wxyz: np.ndarray,
    transform_msg: Optional[Any] = None,
) -> Any:
    """
    Populate a geometry_msgs/Transform message from translation and quaternion
    """

    try:
        from geometry_msgs.msg import Transform as TransformMsg
    except ModuleNotFoundError as exc:
        raise RuntimeError("geometry_msgs is required for to_ros_transform") from exc

    transform_msg = TransformMsg()
    transform_msg.translation.x = float(translation_m[0])
    transform_msg.translation.y = float(translation_m[1])
    transform_msg.translation.z = float(translation_m[2])
    transform_msg.rotation.w = float(quat_wxyz[0])
    transform_msg.rotation.x = float(quat_wxyz[1])
    transform_msg.rotation.y = float(quat_wxyz[2])
    transform_msg.rotation.z = float(quat_wxyz[3])
    return transform_msg


def _skew_symmetric(vector: np.ndarray) -> np.ndarray:
    vector_vals: np.ndarray = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [
            [0.0, -vector_vals[2], vector_vals[1]],
            [vector_vals[2], 0.0, -vector_vals[0]],
            [-vector_vals[1], vector_vals[0], 0.0],
        ],
        dtype=float,
    )
