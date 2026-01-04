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

import numpy as np

from oasis_control.localization.tilt_pose_estimator import ImuCalibration


class SpeedEstimate:
    """
    Scalar forward speed estimate.

    Attributes:
        speed_mps: Estimated forward speed in m/s along the IMU +X axis.
        variance: Variance of the speed estimate in (m/s)^2.
        linear_accel: Latest gravity-compensated acceleration in m/s^2.
        forward_accel_variance: Variance of the forward acceleration
            component in (m/s^2)^2.
    """

    def __init__(
        self,
        speed_mps: float,
        variance: float,
        linear_accel: np.ndarray,
        forward_accel_variance: float,
    ) -> None:
        self.speed_mps: float = speed_mps
        self.variance: float = variance
        self.linear_accel: np.ndarray = linear_accel
        self.forward_accel_variance: float = forward_accel_variance


class SpeedEstimator:
    """
    Integrate calibrated acceleration to estimate forward speed.

    State vector:
        v = forward speed along the IMU +X axis in m/s.

    Process model:
        v_k+1 = v_k + a_x dt, where a_x is gravity-compensated forward
        acceleration.

    Measurement model:
        Optional zero-velocity updates (ZUPT) when the IMU appears
        stationary to softly drive speed toward zero.

    Covariance model:
        Forward speed variance accumulates from forward acceleration
        variance scaled by dt^2.
    """

    def __init__(
        self,
        gravity_mps2: float = 9.80665,
        zero_velocity_accel_threshold_mps2: float = 0.25,
        zero_velocity_gyro_threshold_rads: float = 0.05,
        zero_velocity_count: int = 15,
        zero_velocity_alpha: float = 0.2,
        zero_velocity_clamp_mps: float = 0.05,
    ) -> None:
        self._gravity_mps2: float = gravity_mps2
        self._zero_velocity_accel_threshold_mps2: float = (
            zero_velocity_accel_threshold_mps2
        )
        self._zero_velocity_gyro_threshold_rads: float = (
            zero_velocity_gyro_threshold_rads
        )
        self._zero_velocity_count: int = max(1, zero_velocity_count)
        self._zero_velocity_alpha: float = zero_velocity_alpha
        self._zero_velocity_clamp_mps: float = zero_velocity_clamp_mps

        self._speed_mps: float = 0.0
        self._speed_variance: float = 0.0
        self._stationary_counter: int = 0

    def reset(self) -> None:
        self._speed_mps = 0.0
        self._speed_variance = 0.0
        self._stationary_counter = 0

    def update(
        self,
        linear_accel: Iterable[float],
        angular_velocity: Iterable[float],
        orientation_quaternion: Optional[Iterable[float]],
        orientation_covariance: Optional[Iterable[float]],
        linear_accel_covariance: Optional[Iterable[float]],
        dt_s: float,
        calibration: ImuCalibration,
        gravity_mps2: Optional[float] = None,
    ) -> Optional[SpeedEstimate]:
        accel_values: tuple[float, ...] = tuple(linear_accel)
        gyro_values: tuple[float, ...] = tuple(angular_velocity)

        if len(accel_values) != 3 or len(gyro_values) != 3:
            return None

        accel_raw: np.ndarray = np.array(accel_values, dtype=float)
        gyro: np.ndarray = np.array(gyro_values, dtype=float)

        gravity: float = (
            float(gravity_mps2) if gravity_mps2 is not None else self._gravity_mps2
        )
        self._gravity_mps2 = gravity

        accel_cal: np.ndarray = self._apply_calibration(accel_raw, calibration)

        accel_cov: np.ndarray = self._calibrated_accel_covariance(
            accel_raw, linear_accel_covariance, calibration
        )

        gravity_body: np.ndarray = self._gravity_body_vector(
            orientation_quaternion, orientation_covariance, gravity
        )
        linear_accel_body: np.ndarray = accel_cal - gravity_body

        forward_accel: float = float(linear_accel_body[0])
        forward_var: float = float(max(accel_cov[0, 0], 0.0))

        dt: float = max(0.0, float(dt_s))
        self._speed_mps += forward_accel * dt
        self._speed_variance += forward_var * dt * dt

        self._apply_zero_velocity_detection(linear_accel_body, gyro)

        return SpeedEstimate(
            speed_mps=self._speed_mps,
            variance=self._speed_variance,
            linear_accel=linear_accel_body,
            forward_accel_variance=forward_var,
        )

    def speed(self) -> float:
        return self._speed_mps

    def variance(self) -> float:
        return self._speed_variance

    def _apply_calibration(
        self, accel_raw: np.ndarray, calibration: ImuCalibration
    ) -> np.ndarray:
        if calibration.accel_a.shape != (3, 3):
            return accel_raw

        accel_bias: np.ndarray = calibration.accel_bias_mps2.reshape((3,))
        corrected: np.ndarray = accel_raw - accel_bias
        return calibration.accel_a @ corrected

    def _calibrated_accel_covariance(
        self,
        accel_raw: np.ndarray,
        linear_accel_covariance: Optional[Iterable[float]],
        calibration: ImuCalibration,
    ) -> np.ndarray:
        base: Optional[np.ndarray] = self._covariance_matrix(linear_accel_covariance)
        if base is None:
            base = np.zeros((3, 3), dtype=float)

        if calibration.accel_a.shape == (3, 3):
            base = calibration.accel_a @ base @ calibration.accel_a.T

        param_cov: np.ndarray = self._accel_param_covariance(accel_raw, calibration)
        total: np.ndarray = base + param_cov
        return self._symmetrize(total)

    def _accel_param_covariance(
        self, accel_raw: np.ndarray, calibration: ImuCalibration
    ) -> np.ndarray:
        if calibration.accel_a.shape != (3, 3):
            return np.zeros((3, 3), dtype=float)

        if calibration.accel_param_cov.shape != (12, 12):
            return np.zeros((3, 3), dtype=float)

        accel_a: np.ndarray = calibration.accel_a

        try:
            u: np.ndarray = np.linalg.solve(accel_a, accel_raw)
        except np.linalg.LinAlgError:
            u = np.linalg.pinv(accel_a) @ accel_raw

        jac: np.ndarray = np.zeros((3, 12), dtype=float)
        jac[:, 0:3] = -accel_a

        for row in range(3):
            for col in range(3):
                jac[row, 3 + row * 3 + col] = u[col]

        param_cov: np.ndarray = jac @ calibration.accel_param_cov @ jac.T
        return self._symmetrize(param_cov)

    def _gravity_body_vector(
        self,
        orientation_quaternion: Optional[Iterable[float]],
        orientation_covariance: Optional[Iterable[float]],
        gravity_mps2: float,
    ) -> np.ndarray:
        if not self._orientation_valid(orientation_quaternion, orientation_covariance):
            return np.array([0.0, 0.0, gravity_mps2], dtype=float)

        quat: tuple[float, ...] = tuple(float(value) for value in orientation_quaternion)  # type: ignore[arg-type]
        if len(quat) != 4:
            return np.array([0.0, 0.0, gravity_mps2], dtype=float)

        x, y, z, w = quat

        # Rotation matrix from body to world frame
        r00: float = 1.0 - 2.0 * (y * y + z * z)
        r01: float = 2.0 * (x * y - z * w)
        r02: float = 2.0 * (x * z + y * w)

        r10: float = 2.0 * (x * y + z * w)
        r11: float = 1.0 - 2.0 * (x * x + z * z)
        r12: float = 2.0 * (y * z - x * w)

        r20: float = 2.0 * (x * z - y * w)
        r21: float = 2.0 * (y * z + x * w)
        r22: float = 1.0 - 2.0 * (x * x + y * y)

        rotation: np.ndarray = np.array(
            [
                [r00, r01, r02],
                [r10, r11, r12],
                [r20, r21, r22],
            ],
            dtype=float,
        )

        gravity_world: np.ndarray = np.array([0.0, 0.0, gravity_mps2], dtype=float)
        # Transform gravity from world to body frame
        return rotation.T @ gravity_world

    def _orientation_valid(
        self,
        orientation_quaternion: Optional[Iterable[float]],
        orientation_covariance: Optional[Iterable[float]],
    ) -> bool:
        if orientation_quaternion is None:
            return False

        quat: tuple[float, ...] = tuple(float(value) for value in orientation_quaternion)  # type: ignore[arg-type]
        if len(quat) != 4:
            return False

        if all(value == 0.0 for value in quat):
            return False

        if orientation_covariance is None:
            return True

        cov: tuple[float, ...] = tuple(float(value) for value in orientation_covariance)
        if len(cov) != 9:
            return True

        return cov[0] >= 0.0

    def _apply_zero_velocity_detection(
        self, linear_accel_body: np.ndarray, gyro: np.ndarray
    ) -> None:
        accel_norm: float = float(np.linalg.norm(linear_accel_body))
        gyro_norm: float = float(np.linalg.norm(gyro))

        if (
            accel_norm < self._zero_velocity_accel_threshold_mps2
            and gyro_norm < self._zero_velocity_gyro_threshold_rads
        ):
            self._stationary_counter += 1
        else:
            self._stationary_counter = 0

        if self._stationary_counter < self._zero_velocity_count:
            return

        alpha: float = min(max(self._zero_velocity_alpha, 0.0), 1.0)
        self._speed_mps *= 1.0 - alpha
        self._speed_variance *= (1.0 - alpha) * (1.0 - alpha)

        if abs(self._speed_mps) < self._zero_velocity_clamp_mps:
            self._speed_mps = 0.0

    def _covariance_matrix(
        self, covariance_values: Optional[Iterable[float]]
    ) -> Optional[np.ndarray]:
        if covariance_values is None:
            return None

        data: tuple[float, ...] = tuple(float(value) for value in covariance_values)
        if len(data) != 9:
            return None

        if data[0] < 0.0:
            return None

        matrix: np.ndarray = np.array(data, dtype=float).reshape((3, 3))
        return self._symmetrize(matrix)

    def _symmetrize(self, matrix: np.ndarray) -> np.ndarray:
        return 0.5 * (matrix + matrix.T)
