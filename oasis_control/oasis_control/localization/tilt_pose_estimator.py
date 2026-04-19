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

from oasis_control.localization.common.measurements.tilt_covariance import (
    gravity_covariance_to_tilt_variance_rad2,
)


################################################################################
# Tilt estimator
################################################################################


# Units: m/s^2
# Meaning: reject degenerate gravity vectors during calibration and updates
MIN_GRAVITY_NORM_MPS2: float = 1.0e-3

# Units: rad^2
# Meaning: tiny numerical floor to avoid degenerate zero tilt variance
MIN_TILT_VARIANCE_RAD2: float = 1.0e-12

# Units: rad^2
# Meaning: large yaw variance because gravity does not observe yaw
YAW_VARIANCE_RAD2: float = 1.0e6


@dataclass(frozen=True)
class TiltEstimate:
    """
    Tilt estimate relative to the learned boot gravity direction.

    Fields:
        roll_rad: roll angle in radians relative to the boot reference
        pitch_rad: pitch angle in radians relative to the boot reference
        quaternion_xyzw: tilt-only quaternion in ROS xyzw order
        orientation_covariance: sensor_msgs/Imu 3x3 row-major covariance
    """

    roll_rad: float
    pitch_rad: float
    quaternion_xyzw: tuple[float, float, float, float]
    orientation_covariance: list[float]


class TiltPoseEstimator:
    """
    Estimate gravity-derived tilt with a stationary boot calibration window.

    The first valid gravity samples are averaged while the vehicle is assumed
    stationary. That learned gravity direction defines zero tilt. After
    initialization, the estimator publishes the shortest rotation from the boot
    gravity direction to the current gravity direction. Published roll/pitch
    covariance combines boot calibration scatter with a runtime
    gravity-covariance contribution, with only a tiny numerical floor to avoid
    degenerate zero variance. Yaw remains unobserved.
    """

    def __init__(self, calibration_duration_sec: float) -> None:
        """
        Initialize the tilt estimator.
        """

        self._calibration_duration_sec: float = max(0.0, calibration_duration_sec)
        self._calibration_start_sec: Optional[float] = None
        self._gravity_sum: list[float] = [0.0, 0.0, 0.0]
        self._gravity_samples: list[tuple[float, float, float]] = []
        self._reference_gravity_unit: Optional[tuple[float, float, float]] = None

        # Units: rad^2
        # Meaning: shared roll/pitch variance after calibration
        self._roll_pitch_variance_rad2: float = MIN_TILT_VARIANCE_RAD2

    @property
    def initialized(self) -> bool:
        """
        True once the boot gravity reference has been learned
        """

        return self._reference_gravity_unit is not None

    @property
    def reference_gravity_unit(self) -> Optional[tuple[float, float, float]]:
        """
        Learned unit gravity direction from the stationary boot window
        """

        if self._reference_gravity_unit is None:
            return None

        return (
            float(self._reference_gravity_unit[0]),
            float(self._reference_gravity_unit[1]),
            float(self._reference_gravity_unit[2]),
        )

    def add_calibration_sample(
        self, gravity_mps2: Iterable[float], timestamp_sec: float
    ) -> bool:
        """
        Add a gravity sample during the stationary boot calibration window.
        """

        if self.initialized:
            return True

        gravity_vector: Optional[tuple[float, float, float]] = _normalized_vector(
            gravity_mps2
        )
        if gravity_vector is None:
            return False

        if self._calibration_start_sec is None:
            self._calibration_start_sec = timestamp_sec

        self._gravity_sum[0] += gravity_vector[0]
        self._gravity_sum[1] += gravity_vector[1]
        self._gravity_sum[2] += gravity_vector[2]
        self._gravity_samples.append(gravity_vector)

        elapsed_sec: float = timestamp_sec - self._calibration_start_sec
        if elapsed_sec < self._calibration_duration_sec:
            return False

        reference_gravity_unit: Optional[tuple[float, float, float]] = (
            _normalized_vector(self._gravity_sum)
        )
        if reference_gravity_unit is None:
            return False

        self._reference_gravity_unit = reference_gravity_unit
        self._roll_pitch_variance_rad2 = self._estimate_roll_pitch_variance_rad2(
            reference_gravity_unit
        )
        return True

    def update(
        self,
        gravity_mps2: Iterable[float],
        gravity_covariance_mps2_2: Optional[Iterable[Iterable[float]]] = None,
    ) -> Optional[TiltEstimate]:
        """
        Update the tilt estimate from the current gravity vector.
        """

        if self._reference_gravity_unit is None:
            return None

        gravity_unit: Optional[tuple[float, float, float]] = _normalized_vector(
            gravity_mps2
        )
        if gravity_unit is None:
            return None

        quaternion_xyzw: tuple[float, float, float, float] = (
            _quaternion_from_two_unit_vectors(
                self._reference_gravity_unit,
                gravity_unit,
            )
        )
        roll_rad: float
        pitch_rad: float
        _: float
        roll_rad, pitch_rad, _ = _euler_from_quaternion_xyzw(quaternion_xyzw)
        roll_pitch_variance_rad2: float = max(
            MIN_TILT_VARIANCE_RAD2,
            self._roll_pitch_variance_rad2,
            _gravity_covariance_to_tilt_variance_rad2(
                gravity_mps2=gravity_mps2,
                gravity_covariance_mps2_2=gravity_covariance_mps2_2,
            ),
        )

        return TiltEstimate(
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            quaternion_xyzw=quaternion_xyzw,
            orientation_covariance=[
                roll_pitch_variance_rad2,
                0.0,
                0.0,
                0.0,
                roll_pitch_variance_rad2,
                0.0,
                0.0,
                0.0,
                YAW_VARIANCE_RAD2,
            ],
        )

    def _estimate_roll_pitch_variance_rad2(
        self, reference_gravity_unit: tuple[float, float, float]
    ) -> float:
        """
        Estimate roll/pitch variance from calibration scatter.
        """

        if not self._gravity_samples:
            return MIN_TILT_VARIANCE_RAD2

        residual_angles_rad2: list[float] = []
        sample_gravity_unit: tuple[float, float, float]
        for sample_gravity_unit in self._gravity_samples:
            alignment: float = _clamp(
                _dot(reference_gravity_unit, sample_gravity_unit), -1.0, 1.0
            )
            residual_angle_rad: float = math.acos(alignment)
            residual_angles_rad2.append(residual_angle_rad * residual_angle_rad)

        residual_rms_rad2: float = sum(residual_angles_rad2) / len(residual_angles_rad2)
        return max(MIN_TILT_VARIANCE_RAD2, residual_rms_rad2)


def _normalized_vector(vector: Iterable[float]) -> Optional[tuple[float, float, float]]:
    """
    Return a normalized 3D vector or None for degenerate inputs.
    """

    values: tuple[float, ...] = tuple(float(value) for value in vector)
    if len(values) != 3:
        return None

    norm: float = math.sqrt(sum(value * value for value in values))
    if norm < MIN_GRAVITY_NORM_MPS2:
        return None

    return (
        values[0] / norm,
        values[1] / norm,
        values[2] / norm,
    )


def _gravity_covariance_to_tilt_variance_rad2(
    gravity_mps2: Iterable[float],
    gravity_covariance_mps2_2: Optional[Iterable[Iterable[float]]],
) -> float:
    """
    Backward-compatible wrapper around the shared gravity tilt scale helper.
    """

    return gravity_covariance_to_tilt_variance_rad2(
        gravity_mps2=gravity_mps2,
        gravity_covariance_mps2_2=gravity_covariance_mps2_2,
    )


def _quaternion_from_two_unit_vectors(
    source_unit: tuple[float, float, float], target_unit: tuple[float, float, float]
) -> tuple[float, float, float, float]:
    """
    Compute the shortest rotation from one unit vector to another.
    """

    dot_product: float = _clamp(_dot(source_unit, target_unit), -1.0, 1.0)

    if dot_product > 1.0 - 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)

    if dot_product < -1.0 + 1.0e-9:
        rotation_axis: tuple[float, float, float] = _orthogonal_unit_vector(source_unit)
        return (
            rotation_axis[0],
            rotation_axis[1],
            rotation_axis[2],
            0.0,
        )

    cross_product: tuple[float, float, float] = _cross(source_unit, target_unit)
    quaternion_raw: tuple[float, float, float, float] = (
        cross_product[0],
        cross_product[1],
        cross_product[2],
        1.0 + dot_product,
    )
    quaternion_norm: float = math.sqrt(
        sum(component * component for component in quaternion_raw)
    )
    return (
        quaternion_raw[0] / quaternion_norm,
        quaternion_raw[1] / quaternion_norm,
        quaternion_raw[2] / quaternion_norm,
        quaternion_raw[3] / quaternion_norm,
    )


def _orthogonal_unit_vector(
    vector_unit: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    Choose a stable unit vector orthogonal to the input.
    """

    if abs(vector_unit[0]) < 0.9:
        basis_vector: tuple[float, float, float] = (1.0, 0.0, 0.0)
    else:
        basis_vector = (0.0, 1.0, 0.0)

    orthogonal: tuple[float, float, float] = _cross(vector_unit, basis_vector)
    orthogonal_norm: float = math.sqrt(_dot(orthogonal, orthogonal))
    return (
        orthogonal[0] / orthogonal_norm,
        orthogonal[1] / orthogonal_norm,
        orthogonal[2] / orthogonal_norm,
    )


def _euler_from_quaternion_xyzw(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """
    Convert a quaternion in ROS xyzw order to roll, pitch, yaw.
    """

    qx: float
    qy: float
    qz: float
    qw: float
    qx, qy, qz, qw = quaternion_xyzw

    sinr_cosp: float = 2.0 * (qw * qx + qy * qz)
    cosr_cosp: float = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll_rad: float = math.atan2(sinr_cosp, cosr_cosp)

    sinp: float = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch_rad: float = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)

    siny_cosp: float = 2.0 * (qw * qz + qx * qy)
    cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_rad: float = math.atan2(siny_cosp, cosy_cosp)

    return (roll_rad, pitch_rad, yaw_rad)


def _dot(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    """
    Dot product for 3D vectors.
    """

    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]


def _cross(
    lhs: tuple[float, float, float], rhs: tuple[float, float, float]
) -> tuple[float, float, float]:
    """
    Cross product for 3D vectors.
    """

    return (
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    """
    Clamp a value into a closed interval.
    """

    return max(lower, min(upper, value))
