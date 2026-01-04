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
        imu_is_calibrated: bool = True,
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
        self._imu_is_calibrated: bool = imu_is_calibrated

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

        if self._imu_is_calibrated:
            accel_cal: np.ndarray = accel_raw
        else:
            accel_cal = self._apply_calibration(accel_raw, calibration)

        accel_cov: np.ndarray = self._calibrated_accel_covariance(
            accel_raw, accel_cal, linear_accel_covariance, calibration
        )

        gravity_body: np.ndarray = self._gravity_body_vector(
            orientation_quaternion, orientation_covariance, gravity
        )
        gravity_cov: np.ndarray = self._gravity_projection_covariance(
            orientation_quaternion, orientation_covariance, gravity
        )
        linear_accel_body: np.ndarray = accel_cal - gravity_body

        forward_accel: float = float(linear_accel_body[0])

        accel_lin_cov: np.ndarray = self._symmetrize(accel_cov + gravity_cov)
        forward_var: float = float(max(accel_lin_cov[0, 0], 0.0))

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
        accel_cal: np.ndarray,
        linear_accel_covariance: Optional[Iterable[float]],
        calibration: ImuCalibration,
    ) -> np.ndarray:
        base: Optional[np.ndarray] = self._covariance_matrix(linear_accel_covariance)
        if base is None:
            base = np.zeros((3, 3), dtype=float)

        # Base covariance is already calibrated when the IMU reports
        # calibrated acceleration
        if not self._imu_is_calibrated and calibration.accel_a.shape == (3, 3):
            base = calibration.accel_a @ base @ calibration.accel_a.T

        # Units: (m/s^2)^2. Meaning: parameter-induced covariance in
        # calibrated acceleration.
        param_cov: np.ndarray = self._accel_param_covariance(
            accel_raw, accel_cal, calibration
        )
        total: np.ndarray = base + param_cov
        return self._symmetrize(total)

    def _accel_param_covariance(
        self,
        accel_raw: np.ndarray,
        accel_cal: np.ndarray,
        calibration: ImuCalibration,
    ) -> np.ndarray:
        if calibration.accel_a.shape != (3, 3):
            return np.zeros((3, 3), dtype=float)

        if calibration.accel_param_cov.shape != (12, 12):
            return np.zeros((3, 3), dtype=float)

        accel_a: np.ndarray = calibration.accel_a

        bias: np.ndarray = calibration.accel_bias_mps2.reshape((3,))
        if self._imu_is_calibrated:
            try:
                u: np.ndarray = np.linalg.solve(accel_a, accel_cal)
            except np.linalg.LinAlgError:
                u = np.linalg.pinv(accel_a) @ accel_cal
        else:
            u = accel_raw - bias

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

        quat: Optional[np.ndarray] = self._normalize_quaternion(orientation_quaternion)
        if quat is None:
            return np.array([0.0, 0.0, gravity_mps2], dtype=float)

        x, y, z, w = quat.tolist()

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

    def _gravity_projection_covariance(
        self,
        orientation_quaternion: Optional[Iterable[float]],
        orientation_covariance: Optional[Iterable[float]],
        gravity_mps2: float,
    ) -> np.ndarray:
        orientation_unknown: bool = not self._orientation_valid(
            orientation_quaternion, orientation_covariance
        )

        quat: Optional[np.ndarray] = self._normalize_quaternion(orientation_quaternion)
        if quat is None:
            orientation_unknown = True

        if orientation_unknown or quat is None:
            roll = 0.0
            pitch = 0.0
        else:
            roll, pitch = self._roll_pitch_from_quaternion(quat)

        cov_rp: np.ndarray = self._roll_pitch_covariance(
            orientation_covariance, orientation_unknown
        )

        sin_roll: float = math.sin(roll)
        cos_roll: float = math.cos(roll)
        sin_pitch: float = math.sin(pitch)
        cos_pitch: float = math.cos(pitch)

        gravity: float = float(gravity_mps2)

        jac: np.ndarray = np.zeros((3, 2), dtype=float)

        jac[0, 0] = 0.0
        jac[0, 1] = -gravity * cos_pitch

        jac[1, 0] = gravity * cos_roll * cos_pitch
        jac[1, 1] = -gravity * sin_roll * sin_pitch

        jac[2, 0] = -gravity * sin_roll * cos_pitch
        jac[2, 1] = -gravity * cos_roll * sin_pitch

        return self._symmetrize(jac @ cov_rp @ jac.T)

    def _roll_pitch_from_quaternion(self, quat: np.ndarray) -> tuple[float, float]:
        x, y, z, w = quat.tolist()

        sin_roll: float = 2.0 * (w * x + y * z)
        cos_roll: float = 1.0 - 2.0 * (x * x + y * y)
        roll: float = math.atan2(sin_roll, cos_roll)

        sin_pitch: float = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch: float = math.asin(sin_pitch)

        return roll, pitch

    def _roll_pitch_covariance(
        self,
        orientation_covariance: Optional[Iterable[float]],
        orientation_unknown: bool,
    ) -> np.ndarray:
        if orientation_unknown:
            # Units: rad^2. Meaning: roll/pitch variance when attitude unknown
            variance: float = math.radians(30.0) ** 2
            return np.diag([variance, variance])

        if orientation_covariance is None:
            # Units: rad^2. Meaning: assumed roll/pitch variance when missing
            variance = math.radians(10.0) ** 2
            return np.diag([variance, variance])

        if self._orientation_covariance_unknown(orientation_covariance):
            # Units: rad^2. Meaning: roll/pitch variance when attitude unknown
            variance = math.radians(30.0) ** 2
            return np.diag([variance, variance])

        cov_matrix: Optional[np.ndarray] = self._covariance_matrix(
            orientation_covariance
        )
        if cov_matrix is None:
            # Units: rad^2. Meaning: assumed roll/pitch variance when malformed
            variance = math.radians(10.0) ** 2
            return np.diag([variance, variance])

        return cov_matrix[:2, :2]

    def _normalize_quaternion(
        self, orientation_quaternion: Optional[Iterable[float]]
    ) -> Optional[np.ndarray]:
        if orientation_quaternion is None:
            return None

        quat: tuple[float, ...] = tuple(
            float(value) for value in orientation_quaternion
        )
        if len(quat) != 4:
            return None

        if not all(math.isfinite(value) for value in quat):
            return None

        norm: float = float(np.linalg.norm(np.array(quat, dtype=float)))
        if not math.isfinite(norm) or norm <= 0.0:
            return None

        return np.array(quat, dtype=float) / norm

    def _orientation_valid(
        self,
        orientation_quaternion: Optional[Iterable[float]],
        orientation_covariance: Optional[Iterable[float]],
    ) -> bool:
        if orientation_quaternion is None:
            return False

        quat: Optional[np.ndarray] = self._normalize_quaternion(orientation_quaternion)
        if quat is None:
            return False

        if orientation_covariance is None:
            return True

        return not self._orientation_covariance_unknown(orientation_covariance)

    def _orientation_covariance_unknown(
        self, orientation_covariance: Optional[Iterable[float]]
    ) -> bool:
        if orientation_covariance is None:
            return False

        cov: tuple[float, ...] = tuple(float(value) for value in orientation_covariance)
        if len(cov) != 9:
            return False

        if not all(math.isfinite(value) for value in cov):
            return True

        if cov[0] < 0.0:
            return True

        if any(cov[index] < 0.0 for index in (0, 4, 8)):
            return True

        if self._covariance_matrix(orientation_covariance) is None:
            return True

        return False

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

        if not all(math.isfinite(value) for value in data):
            return None

        if data[0] < 0.0:
            return None

        if any(data[index] < 0.0 for index in (0, 4, 8)):
            return None

        matrix: np.ndarray = np.array(data, dtype=float).reshape((3, 3))
        return self._symmetrize(matrix)

    def _symmetrize(self, matrix: np.ndarray) -> np.ndarray:
        return 0.5 * (matrix + matrix.T)
