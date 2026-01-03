################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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


################################################################################
# Data model
################################################################################


@dataclass
class PoseEstimate:
    """
    Pose estimate derived from inertial measurements.

    Attributes:
        position: Position estimate in meters.
        orientation: Quaternion (x, y, z, w) following ROS conventions.
        covariance: Row-major 6x6 covariance for x, y, z, roll, pitch, yaw.
    """

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    covariance: list[float]


################################################################################
# Online covariance estimation
################################################################################


class OnlineCovarianceEstimator:
    """Track sample covariance for fixed-dimension vectors.

    The estimator implements Welford's online algorithm to accumulate mean and
    covariance without storing the full history of measurements. Only the final
    covariance matrix is exposed to callers.
    """

    def __init__(self, dimension: int, min_variance: float) -> None:
        """
        Initialize the estimator.

        Args:
            dimension: Number of elements in each input vector.
            min_variance: Lower bound for diagonal terms in the estimated
                covariance. Units match the squared units of each vector entry.
        """

        self._dimension = dimension
        self._min_variance = min_variance

        self._count = 0
        self._mean = [0.0] * self._dimension
        self._m2 = [
            [0.0 for _ in range(self._dimension)] for _ in range(self._dimension)
        ]

    def update(self, sample: Iterable[float]) -> None:
        """Update the running covariance with a new sample."""

        values = tuple(sample)
        if len(values) != self._dimension:
            return

        self._count += 1

        delta = [value - mean for value, mean in zip(values, self._mean)]
        inv_count = 1.0 / float(self._count)
        self._mean = [
            mean + delta_value * inv_count
            for mean, delta_value in zip(self._mean, delta)
        ]

        delta2 = [value - mean for value, mean in zip(values, self._mean)]

        for i in range(self._dimension):
            for j in range(self._dimension):
                self._m2[i][j] += delta[i] * delta2[j]

    def covariance(self) -> Optional[tuple[tuple[float, ...], ...]]:
        """Return the unbiased sample covariance if enough samples are present."""

        if self._count < 2:
            return None

        denominator = float(self._count - 1)

        covariance_rows: list[tuple[float, ...]] = []

        for i in range(self._dimension):
            row = []
            for j in range(self._dimension):
                value = self._m2[i][j] / denominator
                if i == j:
                    value = max(value, self._min_variance)
                row.append(value)
            covariance_rows.append(tuple(row))

        return tuple(covariance_rows)


################################################################################
# Tilt estimation with gyro fusion
################################################################################


class TiltPoseEstimator:
    """Estimate tilt using gyro integration and accel correction.

    Roll and pitch are propagated with gyro rates for short-term dynamics and
    corrected with accelerometer-derived gravity for long-term stability. The
    2x2 roll/pitch covariance is tracked and mapped into a full 6x6 pose
    covariance matrix.
    """

    def __init__(
        self,
        gravity_magnitude: float = 9.80665,
        complementary_gain: float = 0.98,
        yaw_variance: float = 1.0e6,
        position_variance: float = 1.0e6,
        min_accel_mps2: float = 1.0e-3,
        min_accel_variance: float = 1.0e-4,
        min_gyro_variance: float = 1.0e-6,
        min_attitude_variance: float = 1.0e-6,
        motion_alpha: float = 1.0,
        motion_deadband_mps2: float = 0.2,
        motion_inflation_max: float = 1.0e3,
    ) -> None:
        """
        Initialize the estimator.

        Args:
            gravity_magnitude: Expected gravity magnitude in m/s^2.
            complementary_gain: Weight of gyro propagation in the fusion
                blend. Smaller values rely more on accelerometer correction.
            yaw_variance: Variance assigned to yaw in rad^2.
            position_variance: Variance assigned to xyz in m^2.
            min_accel_mps2: Minimum acceleration magnitude to accept samples.
            min_accel_variance: Lower bound for accelerometer covariance in
                (m/s^2)^2.
            min_gyro_variance: Lower bound for gyro covariance in (rad/s)^2.
            min_attitude_variance: Lower bound for roll/pitch variance in
                rad^2.
            motion_alpha: Scaling for variance inflation when acceleration
                deviates from the predicted gravity vector.
            motion_deadband_mps2: Residual in m/s^2 ignored before
                motion-based inflation is applied.
            motion_inflation_max: Maximum factor for motion-based covariance
                inflation.
        """

        self._gravity_magnitude = gravity_magnitude
        self._complementary_gain = complementary_gain
        self._yaw_variance = yaw_variance
        self._position_variance = position_variance
        self._min_accel_mps2 = min_accel_mps2
        self._min_gyro_variance = min_gyro_variance
        self._min_attitude_variance = min_attitude_variance
        self._motion_alpha = motion_alpha
        self._motion_deadband_mps2 = motion_deadband_mps2
        self._motion_inflation_max = motion_inflation_max

        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._state_cov = [
            [min_attitude_variance, 0.0],
            [0.0, min_attitude_variance],
        ]

        self._accel_covariance = OnlineCovarianceEstimator(3, min_accel_variance)
        self._gyro_covariance = OnlineCovarianceEstimator(3, min_gyro_variance)

    def update(
        self,
        linear_accel: Iterable[float],
        angular_velocity: Iterable[float],
        dt_s: float,
        linear_accel_covariance: Optional[Iterable[float]] = None,
        angular_velocity_covariance: Optional[Iterable[float]] = None,
    ) -> Optional[PoseEstimate]:
        """
        Update the tilt estimate with accel + gyro data.

        Args:
            linear_accel: Linear acceleration in m/s^2.
            angular_velocity: Angular velocity in rad/s.
            dt_s: Time delta in seconds.
            linear_accel_covariance: 3x3 covariance in row-major form.
            angular_velocity_covariance: 3x3 covariance in row-major form.

        Returns:
            Pose estimate if a valid attitude is available.
        """

        accel_values = tuple(linear_accel)
        gyro_values = tuple(angular_velocity)
        if len(accel_values) != 3 or len(gyro_values) != 3:
            return None

        ax, ay, az = accel_values
        gx, gy, gz = gyro_values

        accel = (float(ax), float(ay), float(az))
        gyro = (float(gx), float(gy), float(gz))

        dt = max(0.0, float(dt_s))

        self._accel_covariance.update(accel)
        self._gyro_covariance.update(gyro)

        accel_magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        has_accel = accel_magnitude >= self._min_accel_mps2

        if self._roll is None or self._pitch is None:
            if not has_accel:
                return None

            self._roll, self._pitch = self._gravity_to_attitude(accel)
            self._state_cov = [
                [self._min_attitude_variance, 0.0],
                [0.0, self._min_attitude_variance],
            ]
        else:
            roll_pred, pitch_pred = self._predict_attitude(gyro, dt)
            pred_cov = self._predict_covariance(
                angular_velocity_covariance,
                dt,
            )

            if has_accel:
                roll_meas, pitch_meas = self._gravity_to_attitude(accel)
                motion_inflation = self._motion_inflation(accel, roll_pred, pitch_pred)
                meas_cov = self._orientation_covariance(
                    accel, linear_accel_covariance, motion_inflation
                )
                roll_upd, pitch_upd, upd_cov = self._complementary_update(
                    (roll_pred, pitch_pred),
                    pred_cov,
                    (roll_meas, pitch_meas),
                    meas_cov,
                )
                self._roll = roll_upd
                self._pitch = pitch_upd
                self._state_cov = upd_cov
            else:
                self._roll = roll_pred
                self._pitch = pitch_pred
                self._state_cov = pred_cov

        orientation = self._rpy_to_quaternion(self._roll, self._pitch, 0.0)
        covariance = self._pose_covariance(self._state_cov)

        return PoseEstimate(
            position=(0.0, 0.0, 0.0),
            orientation=orientation,
            covariance=covariance,
        )

    def _predict_attitude(
        self, gyro: tuple[float, float, float], dt: float
    ) -> tuple[float, float]:
        roll = float(self._roll) if self._roll is not None else 0.0
        pitch = float(self._pitch) if self._pitch is not None else 0.0

        wx, wy, wz = gyro
        tan_pitch = math.tan(pitch)
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)

        roll_rate = wx + sin_roll * tan_pitch * wy + cos_roll * tan_pitch * wz
        pitch_rate = cos_roll * wy - sin_roll * wz

        return roll + roll_rate * dt, pitch + pitch_rate * dt

    def _predict_covariance(
        self,
        angular_velocity_covariance: Optional[Iterable[float]],
        dt: float,
    ) -> list[list[float]]:
        roll = float(self._roll) if self._roll is not None else 0.0
        pitch = float(self._pitch) if self._pitch is not None else 0.0

        gyro_cov = self._gyro_covariance_matrix(angular_velocity_covariance)

        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        tan_pitch = math.tan(pitch)

        jac = (
            (1.0, sin_roll * tan_pitch, cos_roll * tan_pitch),
            (0.0, cos_roll, -sin_roll),
        )

        q = [
            [0.0, 0.0],
            [0.0, 0.0],
        ]

        for i in range(2):
            for j in range(2):
                value = 0.0
                for k in range(3):
                    for col in range(3):
                        value += jac[i][k] * gyro_cov[k][col] * jac[j][col]
                q[i][j] = value * dt * dt

        pred = [
            [self._state_cov[0][0] + q[0][0], self._state_cov[0][1] + q[0][1]],
            [self._state_cov[1][0] + q[1][0], self._state_cov[1][1] + q[1][1]],
        ]

        return self._sanitize_covariance(pred)

    def _gyro_covariance_matrix(
        self,
        angular_velocity_covariance: Optional[Iterable[float]],
    ) -> tuple[tuple[float, ...], ...]:
        covariance_values = self._covariance_values(
            covariance=angular_velocity_covariance,
            fallback=self._gyro_covariance.covariance(),
        )

        if covariance_values is None:
            return (
                (self._min_gyro_variance, 0.0, 0.0),
                (0.0, self._min_gyro_variance, 0.0),
                (0.0, 0.0, self._min_gyro_variance),
            )

        cov = [
            [covariance_values[0], covariance_values[1], covariance_values[2]],
            [covariance_values[3], covariance_values[4], covariance_values[5]],
            [covariance_values[6], covariance_values[7], covariance_values[8]],
        ]

        for i in range(3):
            cov[i][i] = max(cov[i][i], self._min_gyro_variance)

        for i in range(3):
            for j in range(i + 1, 3):
                symmetric = 0.5 * (cov[i][j] + cov[j][i])
                cov[i][j] = symmetric
                cov[j][i] = symmetric

        return (
            (cov[0][0], cov[0][1], cov[0][2]),
            (cov[1][0], cov[1][1], cov[1][2]),
            (cov[2][0], cov[2][1], cov[2][2]),
        )

    def _complementary_update(
        self,
        prediction: tuple[float, float],
        pred_cov: list[list[float]],
        measurement: tuple[float, float],
        meas_cov: list[list[float]],
    ) -> tuple[float, float, list[list[float]]]:
        gain = min(max(self._complementary_gain, 0.0), 1.0)
        inv_gain = 1.0 - gain

        roll = gain * prediction[0] + inv_gain * measurement[0]
        pitch = gain * prediction[1] + inv_gain * measurement[1]

        g2 = gain * gain
        ig2 = inv_gain * inv_gain

        updated = [
            [
                g2 * pred_cov[0][0] + ig2 * meas_cov[0][0],
                g2 * pred_cov[0][1] + ig2 * meas_cov[0][1],
            ],
            [
                g2 * pred_cov[1][0] + ig2 * meas_cov[1][0],
                g2 * pred_cov[1][1] + ig2 * meas_cov[1][1],
            ],
        ]

        return roll, pitch, self._sanitize_covariance(updated)

    def _motion_inflation(
        self, accel: tuple[float, float, float], roll: float, pitch: float
    ) -> float:
        gx = -math.sin(pitch) * self._gravity_magnitude
        gy = math.sin(roll) * math.cos(pitch) * self._gravity_magnitude
        gz = math.cos(roll) * math.cos(pitch) * self._gravity_magnitude

        rx = accel[0] - gx
        ry = accel[1] - gy
        rz = accel[2] - gz
        residual = math.sqrt(rx * rx + ry * ry + rz * rz)

        effective = max(0.0, residual - self._motion_deadband_mps2)
        motion_inflation = (
            1.0 + self._motion_alpha * (effective / self._gravity_magnitude) ** 2
        )
        return min(motion_inflation, self._motion_inflation_max)

    def _orientation_covariance(
        self,
        accel: tuple[float, float, float],
        linear_accel_covariance: Optional[Iterable[float]],
        motion_inflation: float,
    ) -> list[list[float]]:
        covariance_values = self._covariance_values(
            covariance=linear_accel_covariance,
            fallback=self._accel_covariance.covariance(),
        )
        if covariance_values is None:
            base_variance = self._min_attitude_variance
            return [
                [base_variance * motion_inflation, 0.0],
                [0.0, base_variance * motion_inflation],
            ]

        norm = math.sqrt(sum(value * value for value in accel))
        if norm < self._min_accel_mps2:
            base_variance = self._min_attitude_variance
            return [
                [base_variance * motion_inflation, 0.0],
                [0.0, base_variance * motion_inflation],
            ]

        ax, ay, az = accel
        nx = ax / norm
        ny = ay / norm
        nz = az / norm

        eps = 1.0e-12

        d = ny * ny + nz * nz
        d = max(d, eps)

        s = math.sqrt(d)
        q = nx * nx + d
        if q < eps:
            base_variance = self._min_attitude_variance
            return [
                [base_variance * motion_inflation, 0.0],
                [0.0, base_variance * motion_inflation],
            ]

        roll_row_n = (
            0.0,
            nz / d,
            -ny / d,
        )
        pitch_row_n = (
            -s / q,
            (nx * ny) / (s * q),
            (nx * nz) / (s * q),
        )

        inv_norm = 1.0 / max(norm, eps)

        jn = (
            (
                inv_norm * (1.0 - nx * nx),
                inv_norm * (-nx * ny),
                inv_norm * (-nx * nz),
            ),
            (
                inv_norm * (-ny * nx),
                inv_norm * (1.0 - ny * ny),
                inv_norm * (-ny * nz),
            ),
            (
                inv_norm * (-nz * nx),
                inv_norm * (-nz * ny),
                inv_norm * (1.0 - nz * nz),
            ),
        )

        jrp_g = []
        for row in (roll_row_n, pitch_row_n):
            jrp_g.append(
                (
                    row[0] * jn[0][0] + row[1] * jn[1][0] + row[2] * jn[2][0],
                    row[0] * jn[0][1] + row[1] * jn[1][1] + row[2] * jn[2][1],
                    row[0] * jn[0][2] + row[1] * jn[1][2] + row[2] * jn[2][2],
                )
            )

        accel_covariance = (
            (
                covariance_values[0],
                covariance_values[1],
                covariance_values[2],
            ),
            (
                covariance_values[3],
                covariance_values[4],
                covariance_values[5],
            ),
            (
                covariance_values[6],
                covariance_values[7],
                covariance_values[8],
            ),
        )

        covariance_rp = [
            [0.0, 0.0],
            [0.0, 0.0],
        ]

        for i in range(2):
            for j in range(2):
                value = 0.0
                for k in range(3):
                    for col in range(3):
                        value += jrp_g[i][k] * accel_covariance[k][col] * jrp_g[j][col]
                covariance_rp[i][j] = value * motion_inflation

        covariance_rp[0][0] = max(covariance_rp[0][0], self._min_attitude_variance)
        covariance_rp[1][1] = max(covariance_rp[1][1], self._min_attitude_variance)

        return covariance_rp

    def _covariance_values(
        self,
        covariance: Optional[Iterable[float]],
        fallback: Optional[tuple[tuple[float, ...], ...]],
    ) -> Optional[tuple[float, ...]]:
        if covariance is not None:
            covariance_values = tuple(covariance)
            if (
                len(covariance_values) == 9
                and covariance_values[0] >= 0.0
                and any(value != 0.0 for value in covariance_values)
            ):
                return covariance_values

        if fallback is None:
            return None

        return tuple(value for row in fallback for value in row)

    def _pose_covariance(self, tilt_covariance: list[list[float]]) -> list[float]:
        covariance: list[float] = [0.0] * 36
        covariance[0] = self._position_variance
        covariance[7] = self._position_variance
        covariance[14] = self._position_variance

        covariance[21] = tilt_covariance[0][0]
        covariance[22] = tilt_covariance[0][1]
        covariance[27] = tilt_covariance[1][0]
        covariance[28] = tilt_covariance[1][1]

        covariance[35] = self._yaw_variance
        return covariance

    def _sanitize_covariance(self, covariance: list[list[float]]) -> list[list[float]]:
        cov00 = max(covariance[0][0], self._min_attitude_variance)
        cov11 = max(covariance[1][1], self._min_attitude_variance)
        cov01 = 0.5 * (covariance[0][1] + covariance[1][0])
        return [
            [cov00, cov01],
            [cov01, cov11],
        ]

    def _gravity_to_attitude(
        self, gravity_vector: tuple[float, float, float]
    ) -> tuple[float, float]:
        gx, gy, gz = gravity_vector

        norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        if norm < self._min_accel_mps2:
            return 0.0, 0.0

        normalized = (gx / norm, gy / norm, gz / norm)

        roll = math.atan2(normalized[1], normalized[2])
        pitch = math.atan2(
            -normalized[0],
            math.sqrt(normalized[1] ** 2 + normalized[2] ** 2),
        )

        return roll, pitch

    def _rpy_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return x, y, z, w
