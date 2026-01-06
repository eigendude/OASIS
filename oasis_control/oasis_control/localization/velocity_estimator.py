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
Velocity estimation from IMU data using a covariance-aware integrator.

The estimator maintains linear velocity in the world frame and integrates
specific force from the IMU after rotating it into the world frame and removing
gravity. The propagation model is:

    v_{k+1} = v_k + a_world * dt
    P_{k+1} = P_k + dt^2 * cov_a_world

Acceleration covariance is constructed from the IMU measurement covariance,
attitude uncertainty, and a configurable process noise term:

    cov_a_world = R * cov_a_body * R^T
                + [a_world]_x * cov_theta_world * [a_world]_x^T
                + Q_accel_process

Zero-velocity updates (ZUPT) are applied when the system is stationary, as
indicated by both acceleration magnitude near gravity and angular velocity
magnitude near zero. The ZUPT update uses a 3D velocity measurement at zero with
configurable covariance and performs a full Kalman update on velocity and its
covariance.
"""

from __future__ import annotations

from typing import Tuple
from typing import Union

import numpy as np


################################################################################
# Velocity estimator
################################################################################


class VelocityEstimator:
    """
    Integrate IMU linear acceleration into velocity with covariance tracking
    """

    def __init__(
        self,
        gravity_mps2: float,
        accel_process_noise_mps2: Union[float, np.ndarray] = 0.2,
        zupt_enable: bool = True,
        zupt_gyro_threshold_rps: float = 0.15,
        zupt_accel_threshold_mps2: float = 0.2,
        zupt_min_duration_s: float = 0.2,
        zupt_cov_mps: Union[float, np.ndarray] = 0.05,
        max_dt_spike_s: float = 0.2,
    ) -> None:
        """
        Initialize the estimator state and configuration.

        Args:
            gravity_mps2: Gravity magnitude used for gravity subtraction in m/s^2
            accel_process_noise_mps2: Accel process noise std in m/s^2 or 3x3
                covariance
            zupt_enable: Enable zero-velocity updates when stationary
            zupt_gyro_threshold_rps: Gyro magnitude threshold for stationarity in
                rad/s
            zupt_accel_threshold_mps2: Accel magnitude threshold from gravity in
                m/s^2
            zupt_min_duration_s: Minimum stationary duration before applying ZUPT
                in seconds
            zupt_cov_mps: ZUPT velocity measurement std in m/s or 3x3 covariance
            max_dt_spike_s: Maximum delta time to accept before skipping update
        """

        self._gravity_mps2: float = float(gravity_mps2)
        self._accel_process_cov: np.ndarray = self._parse_covariance(
            accel_process_noise_mps2,
            "accel_process_noise_mps2",
        )
        self._zupt_enable: bool = bool(zupt_enable)
        self._zupt_gyro_threshold_rps: float = float(zupt_gyro_threshold_rps)
        self._zupt_accel_threshold_mps2: float = float(zupt_accel_threshold_mps2)
        self._zupt_min_duration_s: float = float(zupt_min_duration_s)
        self._zupt_cov: np.ndarray = self._parse_covariance(
            zupt_cov_mps,
            "zupt_cov_mps",
        )
        self._max_dt_spike_s: float = float(max_dt_spike_s)

        self._v_world: np.ndarray = np.zeros(3, dtype=np.float64)
        self._p_v_world: np.ndarray = np.zeros((3, 3), dtype=np.float64)
        self._stationary_time_s: float = 0.0

    def reset(self) -> None:
        """
        Reset velocity, covariance, and stationary timer to zero
        """

        self._v_world = np.zeros(3, dtype=np.float64)
        self._p_v_world = np.zeros((3, 3), dtype=np.float64)
        self._stationary_time_s = 0.0

    def update(
        self,
        dt_s: float,
        quat_body_to_world: np.ndarray,
        accel_body_mps2: np.ndarray,
        gyro_body_rps: np.ndarray,
        cov_accel_body: np.ndarray,
        cov_theta_body: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, bool]:
        """
        Propagate velocity and covariance with IMU data.

        Args:
            dt_s: Time delta in seconds
            quat_body_to_world: Quaternion (x, y, z, w) rotating body to world
            accel_body_mps2: Linear acceleration in body frame in m/s^2
            gyro_body_rps: Angular velocity in body frame in rad/s
            cov_accel_body: Acceleration covariance in body frame (3x3)
            cov_theta_body: Small-angle attitude covariance in body frame (3x3)

        Returns:
            Tuple of (velocity_body, covariance_body, stationary_flag)
        """

        dt: float = float(dt_s)
        if dt <= 0.0 or dt > self._max_dt_spike_s:
            v_body_skip: np.ndarray
            cov_v_body_skip: np.ndarray
            v_body_skip, cov_v_body_skip = self._to_body_frame(quat_body_to_world)
            return v_body_skip, cov_v_body_skip, self._is_stationary()

        quat_body_to_world_normalized: np.ndarray = self._normalize_quaternion(
            quat_body_to_world
        )
        accel_body: np.ndarray = self._validate_vector(accel_body_mps2, "accel")
        gyro_body: np.ndarray = self._validate_vector(gyro_body_rps, "gyro")
        cov_accel_body_validated: np.ndarray = self._validate_matrix(
            cov_accel_body,
            "cov_accel_body",
        )
        cov_theta_body_validated: np.ndarray = self._validate_matrix(
            cov_theta_body,
            "cov_theta_body",
        )

        rotation_body_to_world: np.ndarray = self._quat_to_rotmat(
            quat_body_to_world_normalized
        )
        rotation_world_to_body: np.ndarray = rotation_body_to_world.T

        accel_world: np.ndarray = rotation_body_to_world @ accel_body

        gravity_world: np.ndarray = np.array(
            [0.0, 0.0, self._gravity_mps2],
            dtype=np.float64,
        )
        accel_linear_world: np.ndarray = accel_world - gravity_world

        # Rotate accel covariance from body frame into world frame
        cov_accel_world: np.ndarray = (
            rotation_body_to_world @ cov_accel_body_validated @ rotation_body_to_world.T
        )

        # Rotate attitude covariance from body frame into world frame
        cov_theta_world: np.ndarray = (
            rotation_body_to_world @ cov_theta_body_validated @ rotation_body_to_world.T
        )

        # Propagate attitude uncertainty into accel covariance
        skew_accel_world: np.ndarray = self._skew(accel_world)
        cov_accel_from_att: np.ndarray = (
            skew_accel_world @ cov_theta_world @ skew_accel_world.T
        )

        # Combine accel covariances with the process noise term
        cov_accel_world_total: np.ndarray = (
            cov_accel_world + cov_accel_from_att + self._accel_process_cov
        )

        self._v_world = self._v_world + accel_linear_world * dt

        # Integrate velocity covariance with constant-accel model
        self._p_v_world = self._p_v_world + cov_accel_world_total * (dt * dt)
        self._p_v_world = self._symmetrize(self._p_v_world)

        stationary: bool = self._update_stationary_state(
            dt=dt,
            accel_body=accel_body,
            gyro_body=gyro_body,
        )

        if stationary and self._zupt_enable:
            self._apply_zupt()

        v_body: np.ndarray = rotation_world_to_body @ self._v_world

        # Rotate velocity covariance from world frame into body frame
        cov_v_body: np.ndarray = (
            rotation_world_to_body @ self._p_v_world @ rotation_world_to_body.T
        )
        cov_v_body = self._symmetrize(cov_v_body)

        return v_body, cov_v_body, stationary

    def _apply_zupt(self) -> None:
        """
        Apply a zero-velocity measurement update in the world frame
        """

        innovation: np.ndarray = -self._v_world

        # Innovation covariance for the ZUPT measurement
        innovation_cov: np.ndarray = self._p_v_world + self._zupt_cov

        # Kalman gain for the ZUPT update
        kalman_gain: np.ndarray = self._p_v_world @ np.linalg.inv(innovation_cov)

        self._v_world = self._v_world + kalman_gain @ innovation

        identity: np.ndarray = np.eye(3, dtype=np.float64)
        temp: np.ndarray = identity - kalman_gain

        # Joseph-form covariance update for numerical stability
        self._p_v_world = (
            temp @ self._p_v_world @ temp.T
            + kalman_gain @ self._zupt_cov @ kalman_gain.T
        )
        self._p_v_world = self._symmetrize(self._p_v_world)

    def _update_stationary_state(
        self,
        dt: float,
        accel_body: np.ndarray,
        gyro_body: np.ndarray,
    ) -> bool:
        """
        Update stationary timer using accel and gyro thresholds
        """

        accel_norm: float = float(np.linalg.norm(accel_body))
        gyro_norm: float = float(np.linalg.norm(gyro_body))

        accel_close_to_gravity: bool = (
            abs(accel_norm - self._gravity_mps2) <= self._zupt_accel_threshold_mps2
        )
        gyro_near_zero: bool = gyro_norm <= self._zupt_gyro_threshold_rps

        if accel_close_to_gravity and gyro_near_zero:
            self._stationary_time_s += dt
        else:
            self._stationary_time_s = 0.0

        return self._is_stationary()

    def _is_stationary(self) -> bool:
        """
        Check if the stationary timer meets the minimum duration
        """

        return self._stationary_time_s >= self._zupt_min_duration_s

    def _to_body_frame(
        self,
        quat_body_to_world: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert current world-frame velocity and covariance into body frame
        """

        quat_norm: np.ndarray = self._normalize_quaternion(quat_body_to_world)
        rotation_body_to_world: np.ndarray = self._quat_to_rotmat(quat_norm)
        rotation_world_to_body: np.ndarray = rotation_body_to_world.T

        v_body: np.ndarray = rotation_world_to_body @ self._v_world

        # Rotate velocity covariance from world frame into body frame
        cov_v_body: np.ndarray = (
            rotation_world_to_body @ self._p_v_world @ rotation_world_to_body.T
        )
        cov_v_body = self._symmetrize(cov_v_body)

        return v_body, cov_v_body

    @staticmethod
    def _normalize_quaternion(quat: np.ndarray) -> np.ndarray:
        """
        Normalize a quaternion to unit length
        """

        quat_array: np.ndarray = np.array(quat, dtype=np.float64).reshape(4)
        norm: float = float(np.linalg.norm(quat_array))
        if norm <= 0.0:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        return quat_array / norm

    @staticmethod
    def _quat_to_rotmat(quat: np.ndarray) -> np.ndarray:
        """
        Convert a unit quaternion (x, y, z, w) to a rotation matrix
        """

        x: float = float(quat[0])
        y: float = float(quat[1])
        z: float = float(quat[2])
        w: float = float(quat[3])

        xx: float = x * x
        yy: float = y * y
        zz: float = z * z
        xy: float = x * y
        xz: float = x * z
        yz: float = y * z
        wx: float = w * x
        wy: float = w * y
        wz: float = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _skew(vector: np.ndarray) -> np.ndarray:
        """
        Build a skew-symmetric matrix from a 3D vector
        """

        vx: float = float(vector[0])
        vy: float = float(vector[1])
        vz: float = float(vector[2])

        return np.array(
            [
                [0.0, -vz, vy],
                [vz, 0.0, -vx],
                [-vy, vx, 0.0],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _symmetrize(matrix: np.ndarray) -> np.ndarray:
        """
        Force symmetry by averaging a matrix with its transpose
        """

        return 0.5 * (matrix + matrix.T)

    @staticmethod
    def _validate_vector(vector: np.ndarray, name: str) -> np.ndarray:
        """
        Validate a 3D vector and cast to float64
        """

        array: np.ndarray = np.array(vector, dtype=np.float64).reshape(-1)
        if array.shape != (3,):
            raise ValueError(f"{name} must be a 3D vector")

        return array

    @staticmethod
    def _validate_matrix(matrix: np.ndarray, name: str) -> np.ndarray:
        """
        Validate a 3x3 matrix and cast to float64
        """

        array: np.ndarray = np.array(matrix, dtype=np.float64)
        if array.shape != (3, 3):
            raise ValueError(f"{name} must be a 3x3 matrix")

        return array

    @staticmethod
    def _parse_covariance(
        value: Union[float, np.ndarray],
        name: str,
    ) -> np.ndarray:
        """
        Parse a covariance from a scalar std or a full 3x3 matrix
        """

        if isinstance(value, np.ndarray):
            return VelocityEstimator._validate_matrix(value, name)

        scalar: float = float(value)

        variance: float = scalar * scalar

        return np.eye(3, dtype=np.float64) * variance
