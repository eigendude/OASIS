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
# Gravity-based pose estimation
################################################################################


class GravityPoseEstimator:
    """Estimate pose from gravity direction.

    The estimator low-pass filters accelerometer readings to recover the gravity
    vector. Roll and pitch are derived from the filtered gravity vector and yaw
    is fixed to zero because IMU-only yaw is unobservable. Pose covariance marks
    yaw as highly uncertain.
    """

    def __init__(
        self,
        gravity_magnitude: float = 9.80665,
        alpha: float = 0.05,
        yaw_variance: float = 1.0e6,
        position_variance: float = 1.0e6,
        min_accel_mps2: float = 1.0e-3,
        min_variance: float = 1.0e-3,
    ) -> None:
        """
        Initialize the estimator.

        Args:
            gravity_magnitude: Expected gravity magnitude in m/s^2.
            alpha: Low-pass filter coefficient for accelerometer smoothing.
            yaw_variance: Variance assigned to yaw in rad^2.
            position_variance: Variance assigned to xyz in m^2.
            min_accel_mps2: Minimum acceleration magnitude to accept samples.
            min_variance: Lower bound for attitude variance in rad^2.
        """

        self._gravity_magnitude = gravity_magnitude
        self._alpha = alpha
        self._yaw_variance = yaw_variance
        self._position_variance = position_variance
        self._min_accel_mps2 = min_accel_mps2
        self._min_variance = min_variance

        self._gravity: Optional[tuple[float, float, float]] = None

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

        accel_tuple = tuple(linear_accel)
        if len(accel_tuple) != 3:
            return None

        magnitude = math.sqrt(sum(value * value for value in accel_tuple))
        if magnitude < self._min_accel_mps2:
            return None

        if self._gravity is None:
            self._gravity = accel_tuple
        else:
            self._gravity = tuple(
                (1.0 - self._alpha) * current + self._alpha * new
                for current, new in zip(self._gravity, accel_tuple)
            )

        roll, pitch = self._gravity_to_attitude(self._gravity)
        covariance = self._covariance_from_accel_covariance(linear_accel_covariance)

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
        self, linear_accel_covariance: Optional[Iterable[float]]
    ) -> list[float]:
        roll_variance, pitch_variance = self._orientation_variance(
            linear_accel_covariance
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
        self, linear_accel_covariance: Optional[Iterable[float]]
    ) -> tuple[float, float]:
        """Project accelerometer covariance into roll and pitch variance.

        The roll variance is computed from the Jacobian of atan2(ay, az) with
        respect to the accelerometer inputs. Pitch variance uses the Jacobian of
        atan2(-ax, sqrt(ay^2 + az^2)). The result is returned in rad^2.
        """
        if linear_accel_covariance is None:
            return self._min_variance, self._min_variance

        covariance_values = tuple(linear_accel_covariance)
        if len(covariance_values) != 9:
            return self._min_variance, self._min_variance

        if self._gravity is None:
            return self._min_variance, self._min_variance

        norm = math.sqrt(sum(value * value for value in self._gravity))
        if norm < self._min_accel_mps2:
            return self._min_variance, self._min_variance

        gx, gy, gz = self._gravity

        roll_denominator = gy * gy + gz * gz
        if roll_denominator < self._min_accel_mps2 * self._min_accel_mps2:
            return self._min_variance, self._min_variance

        roll_row = (
            0.0,
            gz / roll_denominator,
            -gy / roll_denominator,
        )

        sqrt_yz = math.sqrt(roll_denominator)
        pitch_denominator = norm * norm
        if sqrt_yz < self._min_accel_mps2:
            return self._min_variance, self._min_variance

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

        roll_variance = max(roll_variance, self._min_variance)
        pitch_variance = max(pitch_variance, self._min_variance)

        return roll_variance, pitch_variance

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
