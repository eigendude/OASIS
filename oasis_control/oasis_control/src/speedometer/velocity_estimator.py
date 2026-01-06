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

from typing import Tuple

import numpy as np


def _skew_symmetric(vec: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -vec[2], vec[1]],
            [vec[2], 0.0, -vec[0]],
            [-vec[1], vec[0], 0.0],
        ]
    )


def _rotation_from_quaternion(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat

    xx = x * x
    yy = y * y
    zz = z * z
    ww = w * w

    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz],
        ]
    )


class SpeedometerEstimator:
    def __init__(self, gravity_mps2: float) -> None:
        self._gravity_mps2 = gravity_mps2
        self._v_world = np.zeros(3)
        self._p_v_world = np.zeros((3, 3))

    def update(
        self,
        dt_s: float,
        quat_body_to_world: np.ndarray,
        accel_body_mps2: np.ndarray,
        cov_accel_body: np.ndarray,
        cov_theta_body: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        rotation_bw = _rotation_from_quaternion(quat_body_to_world)
        accel_world = rotation_bw @ accel_body_mps2

        gravity_world = np.array([0.0, 0.0, self._gravity_mps2])
        accel_linear_world = accel_world - gravity_world

        cov_accel_world_from_accel = (
            rotation_bw @ cov_accel_body @ rotation_bw.T
        )

        cov_theta_world = rotation_bw @ cov_theta_body @ rotation_bw.T
        accel_skew = _skew_symmetric(accel_world)
        cov_accel_world_from_att = (
            accel_skew @ cov_theta_world @ accel_skew.T
        )

        # (m/s^2)^2 process noise for unmodeled acceleration dynamics
        process_noise_accel_var = 0.0

        cov_accel_world = (
            cov_accel_world_from_accel
            + cov_accel_world_from_att
            + process_noise_accel_var * np.eye(3)
        )

        self._v_world = self._v_world + accel_linear_world * dt_s
        self._p_v_world = self._p_v_world + (dt_s * dt_s) * cov_accel_world

        v_body = rotation_bw.T @ self._v_world
        cov_v_body_from_v = rotation_bw.T @ self._p_v_world @ rotation_bw

        v_body_skew = _skew_symmetric(v_body)
        cov_v_body_from_att = v_body_skew @ cov_theta_body @ v_body_skew.T
        cov_v_body = cov_v_body_from_v + cov_v_body_from_att

        return v_body, cov_v_body
