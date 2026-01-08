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
Magnetometer measurement model helpers for the EKF
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np


class MagMeasurementModel:
    """
    Magnetometer measurement model
    """

    def __init__(self) -> None:
        self._mag_world_t: Optional[np.ndarray] = None
        self._rot_bm: np.ndarray = np.eye(3, dtype=float)

    def set_world_field(self, mag_world_t: list[float]) -> None:
        """
        Set the expected magnetic field vector in world frame
        """

        if len(mag_world_t) != 3:
            raise ValueError("mag_world_t must have 3 elements")
        self._mag_world_t = np.asarray(mag_world_t, dtype=float)

    def set_mag_to_body_rotation(self, rot_bm: np.ndarray) -> None:
        """
        Set the rotation from magnetometer frame to body frame
        """

        if rot_bm.shape != (3, 3):
            raise ValueError("rot_bm must be 3x3")
        self._rot_bm = rot_bm.astype(float)

    def linearize(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute predicted measurement and Jacobian for the magnetometer
        """

        if self._mag_world_t is None:
            mag_world_t: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            mag_world_t = self._mag_world_t

        roll: float = float(state[6])
        pitch: float = float(state[7])
        yaw: float = float(state[8])

        rot_world_from_body: np.ndarray = self._rotation_matrix(roll, pitch, yaw)
        rot_body_from_world: np.ndarray = rot_world_from_body.T

        mag_body: np.ndarray = rot_body_from_world @ mag_world_t
        mag_meas: np.ndarray = self._rot_bm.T @ mag_body

        d_rot_d_roll: np.ndarray
        d_rot_d_pitch: np.ndarray
        d_rot_d_yaw: np.ndarray
        d_rot_d_roll, d_rot_d_pitch, d_rot_d_yaw = self._rotation_derivatives(
            roll, pitch, yaw
        )

        d_body_d_roll: np.ndarray = d_rot_d_roll.T @ mag_world_t
        d_body_d_pitch: np.ndarray = d_rot_d_pitch.T @ mag_world_t
        d_body_d_yaw: np.ndarray = d_rot_d_yaw.T @ mag_world_t

        d_meas_d_roll: np.ndarray = self._rot_bm.T @ d_body_d_roll
        d_meas_d_pitch: np.ndarray = self._rot_bm.T @ d_body_d_pitch
        d_meas_d_yaw: np.ndarray = self._rot_bm.T @ d_body_d_yaw

        h: np.ndarray = np.zeros((3, state.shape[0]), dtype=float)
        h[:, 6] = d_meas_d_roll
        h[:, 7] = d_meas_d_pitch
        h[:, 8] = d_meas_d_yaw

        return mag_meas, h

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr: float = math.cos(roll)
        sr: float = math.sin(roll)
        cp: float = math.cos(pitch)
        sp: float = math.sin(pitch)
        cy: float = math.cos(yaw)
        sy: float = math.sin(yaw)

        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=float,
        )

    def _rotation_derivatives(
        self, roll: float, pitch: float, yaw: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cr: float = math.cos(roll)
        sr: float = math.sin(roll)
        cp: float = math.cos(pitch)
        sp: float = math.sin(pitch)
        cy: float = math.cos(yaw)
        sy: float = math.sin(yaw)

        d_rot_d_roll: np.ndarray = np.array(
            [
                [0.0, cy * sp * cr + sy * sr, -cy * sp * sr + sy * cr],
                [0.0, sy * sp * cr - cy * sr, -sy * sp * sr - cy * cr],
                [0.0, cp * cr, -cp * sr],
            ],
            dtype=float,
        )

        d_rot_d_pitch: np.ndarray = np.array(
            [
                [-cy * sp, cy * cp * sr, cy * cp * cr],
                [-sy * sp, sy * cp * sr, sy * cp * cr],
                [-cp, -sp * sr, -sp * cr],
            ],
            dtype=float,
        )

        d_rot_d_yaw: np.ndarray = np.array(
            [
                [-sy * cp, -sy * sp * sr - cy * cr, -sy * sp * cr + cy * sr],
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [0.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        return d_rot_d_roll, d_rot_d_pitch, d_rot_d_yaw
