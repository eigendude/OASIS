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
from dataclasses import dataclass
from typing import Iterable
from typing import Optional

import numpy as np


################################################################################
# Data model
################################################################################


@dataclass
class ImuCalibration:
    """
    IMU calibration parameters with parameter uncertainty.

    Calibration model:
        a_cal = A @ (a_raw - b_a)
        w_cal = w_raw - b_g

    Parameter covariance layout (12 params):
        [bx, by, bz, a00, a01, a02, a10, a11, a12, a20, a21, a22]

    Per-sample measurement noise covariances come from sensor_msgs/Imu
    and are independent of the calibration parameter uncertainty.

    Attributes:
        gravity_mps2: Gravity magnitude assumed during calibration in m/s^2.
        accel_bias_mps2: Accelerometer bias in m/s^2 for x, y, z axes.
            Included for completeness of the upstream calibration model.
            The bias is already applied in calibrated accel, so it is
            not needed for parameter uncertainty propagation.
        accel_a: 3x3 scale/misalignment matrix in row-major order.
        accel_param_cov: 12x12 covariance for [bx, by, bz, a00..a22].
            Propagated into roll/pitch measurement noise.
        gyro_bias_cov: 3x3 covariance of gyroscope bias in (rad/s)^2.
            Used as process noise when integrating gyro rates.
    """

    gravity_mps2: float
    accel_bias_mps2: np.ndarray
    accel_a: np.ndarray
    accel_param_cov: np.ndarray
    gyro_bias_cov: np.ndarray


################################################################################
# Tilt estimator
################################################################################


class TiltPoseEstimator:
    """
    Estimate roll and pitch from calibrated IMU data.

    State vector:
        x = [roll, pitch]

    Process model:
        Propagate roll and pitch by integrating calibrated gyro rates.
        Process noise Q comes from gyro measurement noise, gyro bias
        uncertainty, and a small bias random-walk term that approximates
        unmodeled drift when bias is not in the state.

    Measurement model:
        Compute roll and pitch from the gravity direction inferred from
        calibrated accelerometer samples. Measurement noise R includes
        accel measurement noise and accel calibration parameter
        uncertainty.

    Accelerometer corrections use soft gating based on | ||a|| - g |.
    Measurement covariance is inflated when acceleration magnitude
    diverges from gravity, yielding smaller Kalman gains under vibration.
    Yaw is unobserved, so orientation covariance includes a large yaw
    variance.

    Covariance outputs:
        Roll/pitch covariance is 2x2. Orientation covariance is a full
        3x3 row-major list suitable for sensor_msgs/Imu with yaw variance
        on the diagonal.
    """

    def __init__(
        self,
        accel_trust_threshold_mps2: float = 1.5,
        yaw_variance_rad2: float = 1.0e6,
        min_attitude_variance: float = 1.0e-4,
        max_accel_inflation: float = 100.0,
        gyro_bias_rw_var_rads2: float = 1.0e-6,
    ) -> None:
        r"""
        Initialize the estimator.

        Args:
            accel_trust_threshold_mps2: Threshold on |\|a\|-g| before
                rejecting accelerometer corrections in m/s^2.
            yaw_variance_rad2: Variance assigned to yaw in rad^2.
            min_attitude_variance: Lower bound for roll/pitch variance in
                rad^2. This is higher than the numerical minimum to keep
                covariance truthful on vibrating platforms.
            max_accel_inflation: Maximum multiplier for accel covariance
                when | ||a|| - g | is large.
            gyro_bias_rw_var_rads2: Extra gyro variance in (rad/s)^2 used
                to approximate unmodeled bias drift and vibration.
        """

        self._accel_trust_threshold_mps2: float = accel_trust_threshold_mps2
        self._yaw_variance_rad2: float = yaw_variance_rad2
        self._min_attitude_variance: float = min_attitude_variance
        self._max_accel_inflation: float = max_accel_inflation
        self._gyro_bias_rw_var_rads2: float = gyro_bias_rw_var_rads2

        self._gravity_mps2: float = 9.80665

        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._state_cov: np.ndarray = np.array(
            [
                [min_attitude_variance, 0.0],
                [0.0, min_attitude_variance],
            ],
            dtype=float,
        )

    def update(
        self,
        linear_accel: Iterable[float],
        angular_velocity: Iterable[float],
        dt_s: float,
        linear_accel_covariance: Optional[Iterable[float]],
        angular_velocity_covariance: Optional[Iterable[float]],
        calibration: ImuCalibration,
    ) -> bool:
        """
        Update roll and pitch using a calibrated IMU sample.

        Args:
            linear_accel: Calibrated acceleration in m/s^2.
            angular_velocity: Calibrated angular velocity in rad/s.
            dt_s: Time delta in seconds.
            linear_accel_covariance: 3x3 row-major covariance for accel.
            angular_velocity_covariance: 3x3 row-major covariance for gyro.
            calibration: Calibration model with parameter uncertainty.

        Returns:
            True when a valid estimate is available.
        """

        accel_values: tuple[float, ...] = tuple(linear_accel)
        gyro_values: tuple[float, ...] = tuple(angular_velocity)
        if len(accel_values) != 3 or len(gyro_values) != 3:
            return False

        accel: np.ndarray = np.array(accel_values, dtype=float)
        gyro: np.ndarray = np.array(gyro_values, dtype=float)

        dt: float = max(0.0, float(dt_s))
        self._gravity_mps2 = float(calibration.gravity_mps2)

        gyro_cov: np.ndarray = self._gyro_covariance(
            angular_velocity_covariance,
            calibration.gyro_bias_cov,
        )
        accel_cov: np.ndarray = self._accel_covariance(
            accel,
            linear_accel_covariance,
            calibration,
        )

        if self._roll is None or self._pitch is None:
            if not self._accel_is_trusted(accel):
                return False

            self._roll, self._pitch = self._gravity_to_attitude(accel)
            init_cov: np.ndarray = self._orientation_covariance(accel, accel_cov)
            self._state_cov = self._sanitize_covariance(init_cov)
            return True

        roll_pred: float
        pitch_pred: float
        roll_pred, pitch_pred = self._predict_attitude(gyro, dt)
        pred_cov: np.ndarray = self._predict_covariance(gyro, gyro_cov, dt)

        roll_meas: float
        pitch_meas: float
        roll_meas, pitch_meas = self._gravity_to_attitude(accel)

        meas_cov: np.ndarray = self._orientation_covariance(accel, accel_cov)
        meas_cov = self._inflate_measurement_covariance(accel, meas_cov)

        roll_upd: float
        pitch_upd: float
        upd_cov: np.ndarray
        roll_upd, pitch_upd, upd_cov = self._kalman_update(
            (roll_pred, pitch_pred),
            pred_cov,
            (roll_meas, pitch_meas),
            meas_cov,
        )
        self._roll = self._wrap_pi(roll_upd)
        self._pitch = self._wrap_pi(pitch_upd)
        self._state_cov = upd_cov

        return True

    def attitude_rpy(self) -> tuple[float, float, float]:
        roll: float = 0.0 if self._roll is None else float(self._roll)
        pitch: float = 0.0 if self._pitch is None else float(self._pitch)
        return self._wrap_pi(roll), self._wrap_pi(pitch), 0.0

    def orientation_quaternion(self) -> tuple[float, float, float, float]:
        roll: float
        pitch: float
        yaw: float
        roll, pitch, yaw = self.attitude_rpy()
        return self._rpy_to_quaternion(roll, pitch, yaw)

    def orientation_covariance_rpy(self) -> list[float]:
        """
        Return roll/pitch covariance with large yaw variance on the diagonal.

        The yaw term remains large because the filter does not observe yaw.
        """

        covariance: np.ndarray = np.zeros((3, 3), dtype=float)
        covariance[0:2, 0:2] = self._state_cov
        covariance[2, 2] = self._yaw_variance_rad2
        return covariance.reshape(-1).tolist()

    def _accel_is_trusted(self, accel: np.ndarray) -> bool:
        magnitude: float = float(np.linalg.norm(accel))

        # Units: m/s^2. Meaning: acceptable deviation from gravity.
        threshold: float = self._accel_trust_threshold_mps2
        return abs(magnitude - self._gravity_mps2) <= threshold

    def _predict_attitude(self, gyro: np.ndarray, dt: float) -> tuple[float, float]:
        roll: float = 0.0 if self._roll is None else float(self._roll)
        pitch: float = 0.0 if self._pitch is None else float(self._pitch)

        roll_rate: float
        pitch_rate: float
        roll_rate, pitch_rate = self._roll_pitch_rate(roll, pitch, gyro)

        pred_roll: float = roll + roll_rate * dt
        pred_pitch: float = pitch + pitch_rate * dt
        return self._wrap_pi(pred_roll), self._wrap_pi(pred_pitch)

    def _predict_covariance(
        self, gyro: np.ndarray, gyro_cov: np.ndarray, dt: float
    ) -> np.ndarray:
        roll: float = 0.0 if self._roll is None else float(self._roll)
        pitch: float = 0.0 if self._pitch is None else float(self._pitch)

        def f(state: np.ndarray, omega: np.ndarray) -> np.ndarray:
            roll_rate: float
            pitch_rate: float
            roll_rate, pitch_rate = self._roll_pitch_rate(
                float(state[0]),
                float(state[1]),
                omega,
            )
            return state + dt * np.array([roll_rate, pitch_rate], dtype=float)

        state: np.ndarray = np.array([roll, pitch], dtype=float)
        omega: np.ndarray = np.array(gyro, dtype=float)

        # Unitless. Meaning: step size for numerical Jacobians.
        epsilon: float = 1.0e-6

        # P = F P F^T + Q with Q = G Cov_w G^T.
        # F = ∂f/∂x and G = ∂f/∂w from the discrete-time state update.
        # The Jacobians are computed with central differences
        jac_f: np.ndarray = np.zeros((2, 2), dtype=float)
        delta: np.ndarray
        f_plus: np.ndarray
        f_minus: np.ndarray
        for idx in range(2):
            delta = np.zeros(2, dtype=float)
            delta[idx] = epsilon
            f_plus = f(state + delta, omega)
            f_minus = f(state - delta, omega)
            jac_f[:, idx] = (f_plus - f_minus) / (2.0 * epsilon)

        jac_g: np.ndarray = np.zeros((2, 3), dtype=float)
        for idx in range(3):
            delta = np.zeros(3, dtype=float)
            delta[idx] = epsilon
            f_plus = f(state, omega + delta)
            f_minus = f(state, omega - delta)
            jac_g[:, idx] = (f_plus - f_minus) / (2.0 * epsilon)

        q: np.ndarray = jac_g @ gyro_cov @ jac_g.T

        pred: np.ndarray = jac_f @ self._state_cov @ jac_f.T + q
        return self._sanitize_covariance(pred)

    def _gyro_covariance(
        self,
        angular_velocity_covariance: Optional[Iterable[float]],
        gyro_bias_cov: np.ndarray,
    ) -> np.ndarray:
        """
        Build gyro covariance used for process noise.

        The bias random-walk term approximates unmodeled bias drift since
        we do not estimate gyro bias as part of the state.
        """

        base: Optional[np.ndarray] = self._covariance_matrix(
            angular_velocity_covariance
        )
        if base is None:
            base = np.zeros((3, 3), dtype=float)

        if gyro_bias_cov.shape != (3, 3):
            bias_cov: np.ndarray = np.zeros((3, 3), dtype=float)
        else:
            bias_cov = gyro_bias_cov

        # Units: (rad/s)^2. Meaning: unmodeled gyro bias drift variance.
        bias_rw: float = max(0.0, float(self._gyro_bias_rw_var_rads2))
        bias_rw_cov: np.ndarray = np.eye(3, dtype=float) * bias_rw

        total: np.ndarray = base + bias_cov + bias_rw_cov
        return self._symmetrize(total)

    def _accel_covariance(
        self,
        accel: np.ndarray,
        linear_accel_covariance: Optional[Iterable[float]],
        calibration: ImuCalibration,
    ) -> np.ndarray:
        base: Optional[np.ndarray] = self._covariance_matrix(linear_accel_covariance)
        if base is None:
            base = np.zeros((3, 3), dtype=float)

        param_cov: np.ndarray = self._accel_param_covariance(accel, calibration)
        total: np.ndarray = base + param_cov
        return self._symmetrize(total)

    def _accel_param_covariance(
        self, accel: np.ndarray, calibration: ImuCalibration
    ) -> np.ndarray:
        if calibration.accel_a.shape != (3, 3):
            return np.zeros((3, 3), dtype=float)

        if calibration.accel_param_cov.shape != (12, 12):
            return np.zeros((3, 3), dtype=float)

        accel_a: np.ndarray = calibration.accel_a

        try:
            u: np.ndarray = np.linalg.solve(accel_a, accel)
        except np.linalg.LinAlgError:
            # Fallback to pseudo-inverse when A is singular
            u = np.linalg.pinv(accel_a) @ accel

        # a_cal = A (a_raw - b), given a_cal we approximate u = A^{-1} a_cal
        # ∂a_cal/∂b = -A and ∂a_cal_i/∂A_ij = u_j
        # Cov_param = J_p Cov_params J_p^T
        jac: np.ndarray = np.zeros((3, 12), dtype=float)
        jac[:, 0:3] = -accel_a

        for row in range(3):
            for col in range(3):
                jac[row, 3 + row * 3 + col] = u[col]

        param_cov: np.ndarray = jac @ calibration.accel_param_cov @ jac.T
        return self._symmetrize(param_cov)

    def _orientation_covariance(
        self, accel: np.ndarray, accel_cov: np.ndarray
    ) -> np.ndarray:
        ax: float
        ay: float
        az: float
        ax, ay, az = accel.tolist()
        d: float = ay * ay + az * az

        # Unitless. Meaning: minimum denominator for stability.
        eps: float = 1.0e-12
        d = max(d, eps)
        s: float = math.sqrt(d)

        q: float = d + ax * ax
        q = max(q, eps)

        # roll  = atan2(ay, az)
        # pitch = atan2(-ax, sqrt(ay^2 + az^2))
        # R = J Cov_a J^T
        j_roll: np.ndarray = np.array([0.0, az / d, -ay / d], dtype=float)
        j_pitch: np.ndarray = np.array(
            [-s / q, (ax * ay) / (s * q), (ax * az) / (s * q)],
            dtype=float,
        )
        jac: np.ndarray = np.vstack((j_roll, j_pitch))

        cov: np.ndarray = jac @ accel_cov @ jac.T
        return self._sanitize_covariance(cov)

    def _inflate_measurement_covariance(
        self, accel: np.ndarray, meas_cov: np.ndarray
    ) -> np.ndarray:
        """
        Inflate accelerometer measurement covariance when dynamics are high.

        Alpha = 1 + (| ||a|| - g | / threshold)^2. This keeps updates smooth
        while reducing their influence under vibration.
        """

        magnitude: float = float(np.linalg.norm(accel))

        # Units: m/s^2. Meaning: acceptable deviation from gravity.
        threshold: float = max(self._accel_trust_threshold_mps2, 1.0e-6)

        err: float = abs(magnitude - self._gravity_mps2)

        # Unitless. Meaning: soft gating factor from acceleration error.
        alpha: float = 1.0 + (err / threshold) ** 2
        alpha = min(max(alpha, 1.0), max(1.0, self._max_accel_inflation))

        inflated: np.ndarray = meas_cov * alpha
        return self._sanitize_covariance(inflated)

    def _covariance_matrix(
        self, covariance: Optional[Iterable[float]]
    ) -> Optional[np.ndarray]:
        if covariance is None:
            return None

        values: tuple[float, ...] = tuple(float(value) for value in covariance)
        if len(values) != 9:
            return None

        if not all(math.isfinite(value) for value in values):
            return None

        diag_indices: tuple[int, int, int] = (0, 4, 8)
        if any(values[index] < 0.0 for index in diag_indices):
            return None

        if not any(value != 0.0 for value in values):
            return None

        matrix: np.ndarray = np.array(values, dtype=float).reshape((3, 3))
        return self._symmetrize(matrix)

    def _kalman_update(
        self,
        prediction: tuple[float, float],
        pred_cov: np.ndarray,
        measurement: tuple[float, float],
        meas_cov: np.ndarray,
    ) -> tuple[float, float, np.ndarray]:
        s: np.ndarray = pred_cov + meas_cov
        inv_s: np.ndarray = self._invert_2x2(s)
        k: np.ndarray = pred_cov @ inv_s

        innovation: np.ndarray = np.array(
            [measurement[0] - prediction[0], measurement[1] - prediction[1]],
            dtype=float,
        )

        updated_state: np.ndarray = np.array(prediction, dtype=float) + k @ innovation
        roll: float = float(updated_state[0])
        pitch: float = float(updated_state[1])

        identity: np.ndarray = np.eye(2, dtype=float)
        i_minus_k: np.ndarray = identity - k

        left_term: np.ndarray = i_minus_k @ pred_cov @ i_minus_k.T
        right_term: np.ndarray = k @ meas_cov @ k.T
        updated_cov: np.ndarray = left_term + right_term

        return roll, pitch, self._sanitize_covariance(updated_cov)

    def _invert_2x2(self, matrix: np.ndarray) -> np.ndarray:
        a: float
        b: float
        c: float
        d: float
        a, b = matrix[0, 0], matrix[0, 1]
        c, d = matrix[1, 0], matrix[1, 1]

        det: float = a * d - b * c

        # Unitless. Meaning: minimum determinant to avoid singular inverse.
        eps: float = 1.0e-9
        if not math.isfinite(det) or abs(det) < eps:
            det = eps if det >= 0.0 else -eps

        inv_det: float = 1.0 / det
        inv: np.ndarray = np.array([[d, -b], [-c, a]], dtype=float) * inv_det
        return inv

    def _sanitize_covariance(self, covariance: np.ndarray) -> np.ndarray:
        cov: np.ndarray = self._symmetrize(covariance)

        cov[0, 0] = max(cov[0, 0], self._min_attitude_variance)
        cov[1, 1] = max(cov[1, 1], self._min_attitude_variance)
        return cov

    def _symmetrize(self, covariance: np.ndarray) -> np.ndarray:
        return 0.5 * (covariance + covariance.T)

    def _gravity_to_attitude(self, accel: np.ndarray) -> tuple[float, float]:
        ax: float
        ay: float
        az: float
        ax, ay, az = accel.tolist()

        norm: float = math.sqrt(ax * ax + ay * ay + az * az)
        if norm <= 0.0:
            return 0.0, 0.0

        roll: float = math.atan2(ay, az)
        pitch: float = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        return roll, pitch

    def _roll_pitch_rate(
        self, roll: float, pitch: float, gyro: np.ndarray
    ) -> tuple[float, float]:
        wx: float
        wy: float
        wz: float
        wx, wy, wz = gyro

        # Guard against Euler singularity when cos(pitch) approaches zero
        if abs(math.cos(pitch)) < 1.0e-6:
            # Clamp pitch as part of the process model so Jacobians match
            pitch = math.copysign((math.pi / 2.0) - 1.0e-6, pitch)

        tan_pitch: float = math.tan(pitch)
        cos_roll: float = math.cos(roll)
        sin_roll: float = math.sin(roll)

        roll_rate: float = wx + sin_roll * tan_pitch * wy + cos_roll * tan_pitch * wz
        pitch_rate: float = cos_roll * wy - sin_roll * wz

        return roll_rate, pitch_rate

    def _wrap_pi(self, angle: float) -> float:
        wrapped: float = (angle + math.pi) % (2.0 * math.pi) - math.pi
        if wrapped <= -math.pi:
            wrapped += 2.0 * math.pi
        return wrapped

    def _rpy_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        cr: float = math.cos(roll * 0.5)
        sr: float = math.sin(roll * 0.5)
        cp: float = math.cos(pitch * 0.5)
        sp: float = math.sin(pitch * 0.5)
        cy: float = math.cos(yaw * 0.5)
        sy: float = math.sin(yaw * 0.5)

        x: float = sr * cp * cy - cr * sp * sy
        y: float = cr * sp * cy + sr * cp * sy
        z: float = cr * cp * sy - sr * sp * cy
        w: float = cr * cp * cy + sr * sp * sy

        return x, y, z, w
