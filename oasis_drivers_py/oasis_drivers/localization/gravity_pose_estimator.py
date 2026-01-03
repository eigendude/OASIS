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
import random
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
# Gravity-based pose estimation
################################################################################


class GravityPoseEstimator:
    """Estimate pose from gravity direction.

    The estimator low-pass filters accelerometer readings to recover the gravity
    vector. Roll and pitch are derived from the filtered gravity vector and yaw
    is fixed to zero because IMU-only yaw is unobservable. Pose covariance marks
    yaw as highly uncertain and inflates roll/pitch variance when acceleration
    deviates from gravity.
    """

    def __init__(
        self,
        gravity_magnitude: float = 9.80665,
        alpha: float = 0.05,
        yaw_variance: float = 1.0e6,
        position_variance: float = 1.0e6,
        min_accel_mps2: float = 1.0e-3,
        min_accel_variance: float = 1.0e-4,
        min_attitude_variance: float = 1.0e-6,
        motion_alpha: float = 1.0,
        motion_deadband_mps2: float = 0.2,
    ) -> None:
        """
        Initialize the estimator.

        Args:
            gravity_magnitude: Expected gravity magnitude in m/s^2.
            alpha: Low-pass filter coefficient for accelerometer smoothing.
            yaw_variance: Variance assigned to yaw in rad^2.
            position_variance: Variance assigned to xyz in m^2.
            min_accel_mps2: Minimum acceleration magnitude to accept samples.
            min_accel_variance: Lower bound for accelerometer covariance in
                (m/s^2)^2.
            min_attitude_variance: Lower bound for roll/pitch variance in
                rad^2.
            motion_alpha: Scaling for variance inflation when motion deviates
                from gravity.
            motion_deadband_mps2: Magnitude deviation in m/s^2 ignored before
                motion-based inflation is applied.
        """

        self._gravity_magnitude = gravity_magnitude
        self._alpha = alpha
        self._yaw_variance = yaw_variance
        self._position_variance = position_variance
        self._min_accel_mps2 = min_accel_mps2
        self._min_attitude_variance = min_attitude_variance
        self._motion_alpha = motion_alpha
        self._motion_deadband_mps2 = motion_deadband_mps2

        self._gravity: Optional[tuple[float, float, float]] = None
        self._accel_covariance = OnlineCovarianceEstimator(
            3, min_accel_variance
        )

    def update(
        self,
        linear_accel: Iterable[float],
        linear_accel_covariance: Optional[Iterable[float]] = None,
    ) -> Optional[PoseEstimate]:
        """
        Update the estimate with a new accelerometer sample.

        Args:
            linear_accel: Linear acceleration in m/s^2.
            linear_accel_covariance: 3x3 covariance in row-major form.

        Returns:
            Pose estimate if a valid gravity vector is available.
        """

        accel_values = tuple(linear_accel)
        if len(accel_values) != 3:
            return None

        ax, ay, az = accel_values
        accel: tuple[float, float, float] = (float(ax), float(ay), float(az))

        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        if magnitude < self._min_accel_mps2:
            return None

        deviation = abs(magnitude - self._gravity_magnitude)
        effective = max(0.0, deviation - self._motion_deadband_mps2)
        motion_inflation = 1.0 + self._motion_alpha * (
            effective / self._gravity_magnitude
        ) ** 2

        if self._gravity is None:
            gravity: tuple[float, float, float] = accel
        else:
            gx, gy, gz = self._gravity
            a = self._alpha
            gravity = (
                (1.0 - a) * gx + a * ax,
                (1.0 - a) * gy + a * ay,
                (1.0 - a) * gz + a * az,
            )

        self._gravity = gravity

        self._accel_covariance.update(accel)

        roll, pitch = self._gravity_to_attitude(gravity)
        covariance = self._covariance_from_accel_covariance(
            linear_accel_covariance, motion_inflation
        )

        orientation = self._rpy_to_quaternion(roll, pitch, 0.0)

        return PoseEstimate(
            position=(0.0, 0.0, 0.0),
            orientation=orientation,
            covariance=covariance,
        )

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

    def _covariance_from_accel_covariance(
        self,
        linear_accel_covariance: Optional[Iterable[float]],
        motion_inflation: float,
    ) -> list[float]:
        roll_variance, pitch_variance = self._orientation_variance(
            linear_accel_covariance, motion_inflation
        )

        covariance: list[float] = [0.0] * 36
        covariance[0] = self._position_variance
        covariance[7] = self._position_variance
        covariance[14] = self._position_variance
        covariance[21] = roll_variance
        covariance[28] = pitch_variance
        covariance[35] = self._yaw_variance

        return covariance

    def _orientation_variance(
        self,
        linear_accel_covariance: Optional[Iterable[float]],
        motion_inflation: float,
    ) -> tuple[float, float]:
        """Project accelerometer covariance into roll and pitch variance.

        The roll variance is computed from the Jacobian of atan2(ay, az) with
        respect to the accelerometer inputs. Pitch variance uses the Jacobian of
        atan2(-ax, sqrt(ay^2 + az^2)). The result is returned in rad^2 and
        inflated to reflect motion if the measured acceleration magnitude
        deviates from gravity.
        """
        covariance_values = self._accel_covariance_values(linear_accel_covariance)
        if covariance_values is None:
            base_variance = self._min_attitude_variance
            return (
                base_variance * motion_inflation,
                base_variance * motion_inflation,
            )

        if self._gravity is None:
            base_variance = self._min_attitude_variance
            return (
                base_variance * motion_inflation,
                base_variance * motion_inflation,
            )

        norm = math.sqrt(sum(value * value for value in self._gravity))
        if norm < self._min_accel_mps2:
            base_variance = self._min_attitude_variance
            return (
                base_variance * motion_inflation,
                base_variance * motion_inflation,
            )

        gx, gy, gz = self._gravity

        roll_denominator = gy * gy + gz * gz
        if roll_denominator < self._min_accel_mps2 * self._min_accel_mps2:
            base_variance = self._min_attitude_variance
            return (
                base_variance * motion_inflation,
                base_variance * motion_inflation,
            )

        roll_row = (
            0.0,
            gz / roll_denominator,
            -gy / roll_denominator,
        )

        sqrt_yz = math.sqrt(roll_denominator)
        pitch_denominator = norm * norm
        if sqrt_yz < self._min_accel_mps2:
            base_variance = self._min_attitude_variance
            return (
                base_variance * motion_inflation,
                base_variance * motion_inflation,
            )

        pitch_row = (
            -sqrt_yz / pitch_denominator,
            gx * gy / (sqrt_yz * pitch_denominator),
            gx * gz / (sqrt_yz * pitch_denominator),
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

        roll_variance = self._project_variance(roll_row, accel_covariance)
        pitch_variance = self._project_variance(pitch_row, accel_covariance)

        roll_variance = max(roll_variance, self._min_attitude_variance)
        pitch_variance = max(pitch_variance, self._min_attitude_variance)

        return (
            roll_variance * motion_inflation,
            pitch_variance * motion_inflation,
        )

    def _accel_covariance_values(
        self, linear_accel_covariance: Optional[Iterable[float]]
    ) -> Optional[tuple[float, ...]]:
        """Select the best available accelerometer covariance estimate."""

        if linear_accel_covariance is not None:
            covariance_values = tuple(linear_accel_covariance)
            if (
                len(covariance_values) == 9
                and covariance_values[0] >= 0.0
                and any(value != 0.0 for value in covariance_values)
            ):
                return covariance_values

        covariance_matrix = self._accel_covariance.covariance()
        if covariance_matrix is None:
            return None

        return tuple(value for row in covariance_matrix for value in row)

    def _project_variance(
        self,
        transform_row: tuple[float, float, float],
        covariance: tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ],
    ) -> float:
        """Project a covariance matrix with a single Jacobian row."""
        projected = 0.0

        for i in range(3):
            for j in range(3):
                projected += transform_row[i] * covariance[i][j] * transform_row[j]

        return projected


if __name__ == "__main__":
    random.seed(0)

    estimator = GravityPoseEstimator()
    latest_estimate: Optional[PoseEstimate] = None

    for _ in range(200):
        noisy_gravity = (
            random.gauss(0.0, 0.02),
            random.gauss(0.0, 0.02),
            estimator._gravity_magnitude + random.gauss(0.0, 0.02),
        )
        estimate = estimator.update(noisy_gravity)
        if estimate is not None:
            latest_estimate = estimate

    if latest_estimate is not None:
        print(
            f"Stationary roll variance: {latest_estimate.covariance[21]:.6e}"
        )
        print(
            f"Stationary pitch variance: {latest_estimate.covariance[28]:.6e}"
        )

    for _ in range(50):
        accelerating = (
            2.0 + random.gauss(0.0, 0.05),
            random.gauss(0.0, 0.05),
            estimator._gravity_magnitude + random.gauss(0.0, 0.05),
        )
        estimate = estimator.update(accelerating)
        if estimate is not None:
            latest_estimate = estimate

    if latest_estimate is not None:
        print(
            f"Motion roll variance: {latest_estimate.covariance[21]:.6e}"
        )
        print(
            f"Motion pitch variance: {latest_estimate.covariance[28]:.6e}"
        )
