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
from typing import Optional

from oasis_control.localization.common.algebra.quat import Quaternion
from oasis_control.localization.common.algebra.quat import Vector3
from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import make_mounting_transform


# Units: m/s^2
# Meaning: reject degenerate gravity vectors during the boot solve
MIN_GRAVITY_NORM_MPS2: float = 1.0e-3

# Units: rad/s
# Meaning: default low-rate threshold for accepting stationary boot samples
DEFAULT_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS: float = 0.35

# Units: s
# Meaning: default boot window duration for the gravity mounting solve
DEFAULT_CALIBRATION_DURATION_SEC: float = 2.0

# Meaning: require at least this many accepted gravity samples before solving
DEFAULT_MIN_SAMPLE_COUNT: int = 10


@dataclass(frozen=True)
class BootMountingSolution:
    """
    Fixed `T_BI` rotation solved from the boot gravity window.

    Fields:
        mounting_transform: solved fixed `q_BI` rotation from `imu_link` to
            `base_link`, published in TF as parent=`base_link`,
            child=`imu_link`
        mean_gravity_unit_imu: average unit gravity direction in `imu_link`
            over the accepted boot window
        roll_rad: solved mounting roll in radians
        pitch_rad: solved mounting pitch in radians
        yaw_rad: solved mounting yaw in radians under policy
        sample_count: accepted gravity sample count used in the solve
    """

    mounting_transform: MountingTransform
    mean_gravity_unit_imu: Vector3
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    sample_count: int


class BootMountingCalibrator:
    """
    Solve a fixed IMU-to-body tilt mounting from stationary boot gravity.

    The solver averages accepted unit gravity samples in `imu_link`, solves the
    roll/pitch rotation that maps that average direction onto the expected
    level body-frame gravity direction `(0, 0, -1)`, and fixes mounting yaw to
    zero by policy because gravity does not observe yaw. The solved quaternion
    is therefore `q_BI`, not its inverse.
    """

    def __init__(
        self,
        *,
        parent_frame_id: str,
        child_frame_id: str,
        calibration_duration_sec: float = DEFAULT_CALIBRATION_DURATION_SEC,
        stationary_angular_speed_threshold_rads: float = (
            DEFAULT_STATIONARY_ANGULAR_SPEED_THRESHOLD_RADS
        ),
        min_sample_count: int = DEFAULT_MIN_SAMPLE_COUNT,
    ) -> None:
        self._parent_frame_id: str = parent_frame_id
        self._child_frame_id: str = child_frame_id
        self._calibration_duration_sec: float = max(0.0, calibration_duration_sec)
        self._stationary_angular_speed_threshold_rads: float = max(
            0.0, stationary_angular_speed_threshold_rads
        )
        self._min_sample_count: int = max(1, min_sample_count)
        self._calibration_start_ns: Optional[int] = None
        self._gravity_sum_imu: list[float] = [0.0, 0.0, 0.0]
        self._accepted_sample_count: int = 0
        self._solution: Optional[BootMountingSolution] = None

    @property
    def solution(self) -> Optional[BootMountingSolution]:
        """
        Return the solved fixed mounting when the boot window is complete
        """

        return self._solution

    def add_gravity_sample(
        self,
        gravity_sample: GravitySample,
        angular_velocity_rads: Optional[Vector3] = None,
    ) -> Optional[BootMountingSolution]:
        """
        Add one accepted boot gravity sample to the mounting solve.
        """

        if self._solution is not None:
            return self._solution

        if not _is_stationary(
            angular_velocity_rads,
            self._stationary_angular_speed_threshold_rads,
        ):
            return None

        gravity_unit_imu: Optional[Vector3] = _normalized_vector(
            gravity_sample.gravity_mps2
        )
        if gravity_unit_imu is None:
            return None

        if self._calibration_start_ns is None:
            self._calibration_start_ns = gravity_sample.timestamp_ns

        self._gravity_sum_imu[0] += gravity_unit_imu[0]
        self._gravity_sum_imu[1] += gravity_unit_imu[1]
        self._gravity_sum_imu[2] += gravity_unit_imu[2]
        self._accepted_sample_count += 1

        elapsed_sec: float = (
            float(gravity_sample.timestamp_ns - self._calibration_start_ns) * 1.0e-9
        )
        if elapsed_sec < self._calibration_duration_sec:
            return None

        if self._accepted_sample_count < self._min_sample_count:
            return None

        mean_gravity_unit_imu: Optional[Vector3] = _normalized_vector(
            self._gravity_sum_imu
        )
        if mean_gravity_unit_imu is None:
            return None

        roll_rad, pitch_rad = _solve_mounting_roll_pitch(
            mean_gravity_unit_imu=mean_gravity_unit_imu
        )

        # Units: rad
        # Meaning: gravity does not observe mounting yaw, so fix it by policy
        yaw_rad: float = 0.0

        quaternion_xyzw: Optional[Quaternion] = normalize_quaternion_xyzw(
            _quaternion_from_roll_pitch_yaw(
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                yaw_rad=yaw_rad,
            )
        )
        if quaternion_xyzw is None:
            return None

        self._solution = BootMountingSolution(
            mounting_transform=make_mounting_transform(
                parent_frame_id=self._parent_frame_id,
                child_frame_id=self._child_frame_id,
                quaternion_xyzw=quaternion_xyzw,
            ),
            mean_gravity_unit_imu=mean_gravity_unit_imu,
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_rad,
            sample_count=self._accepted_sample_count,
        )
        return self._solution


def _is_stationary(
    angular_velocity_rads: Optional[Vector3],
    stationary_angular_speed_threshold_rads: float,
) -> bool:
    """
    Reject samples when the latest body rate exceeds the stationary threshold
    """

    if angular_velocity_rads is None:
        return True

    angular_speed_rads: float = math.sqrt(
        angular_velocity_rads[0] * angular_velocity_rads[0]
        + angular_velocity_rads[1] * angular_velocity_rads[1]
        + angular_velocity_rads[2] * angular_velocity_rads[2]
    )
    return angular_speed_rads <= stationary_angular_speed_threshold_rads


def _normalized_vector(vector: Vector3 | list[float]) -> Optional[Vector3]:
    """
    Normalize a 3D vector and reject degenerate inputs
    """

    x_value: float = float(vector[0])
    y_value: float = float(vector[1])
    z_value: float = float(vector[2])
    norm_mps2: float = math.sqrt(
        x_value * x_value + y_value * y_value + z_value * z_value
    )
    if norm_mps2 < MIN_GRAVITY_NORM_MPS2:
        return None

    return (
        x_value / norm_mps2,
        y_value / norm_mps2,
        z_value / norm_mps2,
    )


def _solve_mounting_roll_pitch(
    mean_gravity_unit_imu: Vector3,
) -> tuple[float, float]:
    """
    Solve the roll/pitch part of `R_BI` that levels the boot gravity vector.
    """

    gravity_x_imu: float = mean_gravity_unit_imu[0]
    gravity_y_imu: float = mean_gravity_unit_imu[1]
    gravity_z_imu: float = mean_gravity_unit_imu[2]

    # Units: rad
    # Meaning: roll that removes the lateral gravity component in `imu_link`
    roll_rad: float = math.atan2(-gravity_y_imu, -gravity_z_imu)

    sin_roll: float = math.sin(roll_rad)
    cos_roll: float = math.cos(roll_rad)

    # Units: unitless
    # Meaning: IMU-frame gravity Z after the solved mounting roll is removed
    gravity_z_after_roll: float = sin_roll * gravity_y_imu + cos_roll * gravity_z_imu

    # Units: rad
    # Meaning: pitch that removes the forward gravity component after roll
    pitch_rad: float = math.atan2(gravity_x_imu, -gravity_z_after_roll)

    return (roll_rad, pitch_rad)


def _quaternion_from_roll_pitch_yaw(
    *,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> Quaternion:
    """
    Convert roll, pitch, yaw into a quaternion in ROS xyzw order.
    """

    half_roll_rad: float = roll_rad * 0.5
    half_pitch_rad: float = pitch_rad * 0.5
    half_yaw_rad: float = yaw_rad * 0.5

    sin_half_roll: float = math.sin(half_roll_rad)
    cos_half_roll: float = math.cos(half_roll_rad)
    sin_half_pitch: float = math.sin(half_pitch_rad)
    cos_half_pitch: float = math.cos(half_pitch_rad)
    sin_half_yaw: float = math.sin(half_yaw_rad)
    cos_half_yaw: float = math.cos(half_yaw_rad)

    return (
        sin_half_roll * cos_half_pitch * cos_half_yaw
        - cos_half_roll * sin_half_pitch * sin_half_yaw,
        cos_half_roll * sin_half_pitch * cos_half_yaw
        + sin_half_roll * cos_half_pitch * sin_half_yaw,
        cos_half_roll * cos_half_pitch * sin_half_yaw
        - sin_half_roll * sin_half_pitch * cos_half_yaw,
        cos_half_roll * cos_half_pitch * cos_half_yaw
        + sin_half_roll * sin_half_pitch * sin_half_yaw,
    )
