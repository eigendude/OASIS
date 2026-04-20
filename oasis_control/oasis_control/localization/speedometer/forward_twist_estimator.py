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
import time
from typing import Optional

from oasis_control.localization.speedometer.contracts import ForwardAxisState
from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.localization.speedometer.contracts import ForwardTwistEstimate
from oasis_control.localization.speedometer.contracts import LearningState
from oasis_control.localization.speedometer.contracts import PersistenceRecord
from oasis_control.localization.speedometer.contracts import ZuptMeasurement
from oasis_control.localization.speedometer.forward_yaw_persistence import (
    ForwardYawPersistence,
)
from oasis_control.localization.speedometer.turn_detector import TurnDetector


################################################################################
# Forward twist estimator
################################################################################


MIN_DIRECTION_NORM: float = 1.0e-6


class ForwardTwistEstimator:
    """
    Estimate a learned forward axis plus signed scalar forward speed.

    The estimator keeps candidate and committed forward-axis evidence
    separately. Uncommitted evidence may be discarded when turn detection says
    the current motion is no longer suitable for learning one fixed axis in
    `base_link`.
    """

    def __init__(
        self,
        *,
        config: ForwardTwistConfig,
        persistence: ForwardYawPersistence,
        turn_detector: TurnDetector,
    ) -> None:
        """Initialize the forward-twist estimator."""

        self._config: ForwardTwistConfig = config
        self._persistence: ForwardYawPersistence = persistence
        self._turn_detector: TurnDetector = turn_detector

        self._candidate_forward_yaw_rad: float = 0.0
        self._committed_forward_yaw_rad: float = 0.0
        self._candidate_sample_count: int = 0
        self._committed_sample_count: int = 0
        self._uncommitted_sample_count: int = 0
        self._candidate_confidence: float = 0.0
        self._checkpoint_commit_count: int = 0
        self._checkpoint_discard_count: int = 0
        self._fit_sample_count: int = 0

        self._committed_direction_sum_xy: tuple[float, float] = (0.0, 0.0)
        self._uncommitted_direction_sum_xy: tuple[float, float] = (0.0, 0.0)
        self._forward_sign_reference_xy: Optional[tuple[float, float]] = None

        self._forward_speed_mps: float = 0.0
        self._forward_speed_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            self._config.initial_forward_speed_sigma_mps**2,
        )

        self._last_turn_detected: bool = False
        self._last_stationary_flag: Optional[bool] = None
        self._last_timestamp_ns: int = 0
        self._last_imu_timestamp_ns: int = 0
        self._last_zupt_timestamp_ns: int = 0
        self._have_imu_timestamp: bool = False
        self._have_zupt_timestamp: bool = False
        self._imu_drop_count: int = 0
        self._zupt_drop_count: int = 0
        self._persistence_failure_count: int = 0
        self._persistence_success_count: int = 0
        self._last_persistence_error: str = ""

    @property
    def persistence_success_count(self) -> int:
        """Return the number of successful checkpoint persistence writes."""

        return self._persistence_success_count

    @property
    def persistence_failure_count(self) -> int:
        """Return the number of failed checkpoint persistence writes."""

        return self._persistence_failure_count

    @property
    def last_persistence_error(self) -> str:
        """Return the latest persistence failure text, if any."""

        return self._last_persistence_error

    @property
    def imu_drop_count(self) -> int:
        """Return the number of rejected or dropped IMU samples."""

        return self._imu_drop_count

    @property
    def zupt_drop_count(self) -> int:
        """Return the number of ignored or out-of-order ZUPT samples."""

        return self._zupt_drop_count

    def update_imu(
        self,
        *,
        timestamp_ns: int,
        orientation_xyzw: tuple[float, float, float, float],
        angular_velocity_rads: tuple[float, float, float],
        linear_acceleration_mps2: tuple[float, float, float],
    ) -> ForwardTwistEstimate:
        """
        Update the estimator from one mounted AHRS IMU sample.
        """

        imu_sample_rejected: bool = False
        self._last_timestamp_ns = int(timestamp_ns)

        if not _quaternion_is_finite(orientation_xyzw):
            self._imu_drop_count += 1
            return self._build_estimate(
                imu_sample_rejected=True,
                zupt_sample_rejected=False,
                checkpoint_just_committed=False,
            )

        if self._have_imu_timestamp and timestamp_ns <= self._last_imu_timestamp_ns:
            self._imu_drop_count += 1
            self._last_timestamp_ns = self._last_imu_timestamp_ns
            return self._build_estimate(
                imu_sample_rejected=True,
                zupt_sample_rejected=False,
                checkpoint_just_committed=False,
            )

        dt_sec: Optional[float] = None
        if self._have_imu_timestamp:
            dt_sec = max(
                0.0,
                float(timestamp_ns - self._last_imu_timestamp_ns) * 1.0e-9,
            )
            if dt_sec > self._config.max_imu_dt_sec:
                dt_sec = self._config.max_imu_dt_sec

        self._last_imu_timestamp_ns = int(timestamp_ns)
        self._have_imu_timestamp = True

        turn_detection = self._turn_detector.update(
            angular_velocity_rads=angular_velocity_rads,
            linear_acceleration_mps2=linear_acceleration_mps2,
        )
        self._last_turn_detected = turn_detection.turn_detected

        checkpoint_just_committed: bool = False
        if turn_detection.turn_detected:
            self._discard_uncommitted_learning()
        else:
            checkpoint_just_committed = self._update_learning(
                orientation_xyzw=orientation_xyzw,
                linear_acceleration_mps2=linear_acceleration_mps2,
            )

        if dt_sec is not None and dt_sec > 0.0 and self._committed_sample_count > 0:
            forward_axis_xy: tuple[float, float] = _yaw_to_axis_xy(
                self._committed_forward_yaw_rad
            )
            projected_forward_accel_mps2: float = forward_axis_xy[0] * float(
                linear_acceleration_mps2[0]
            ) + forward_axis_xy[1] * float(linear_acceleration_mps2[1])
            self._forward_speed_mps += projected_forward_accel_mps2 * dt_sec
            self._forward_speed_variance_mps2 = max(
                self._config.min_forward_speed_variance_mps2,
                self._forward_speed_variance_mps2
                + dt_sec
                * dt_sec
                * self._config.forward_accel_process_sigma_mps2
                * self._config.forward_accel_process_sigma_mps2,
            )

        if checkpoint_just_committed and self._config.persistence_write_on_checkpoint:
            self.store_committed_forward_yaw(hostname=self._persistence.hostname)

        return self._build_estimate(
            imu_sample_rejected=imu_sample_rejected,
            zupt_sample_rejected=False,
            checkpoint_just_committed=checkpoint_just_committed,
        )

    def update_zupt(self, *, measurement: ZuptMeasurement) -> ForwardTwistEstimate:
        """
        Apply a scalar zero-velocity update when the paired ZUPT flag is true.
        """

        self._last_stationary_flag = measurement.stationary_flag
        self._last_timestamp_ns = int(measurement.timestamp_ns)

        if (
            self._have_zupt_timestamp
            and measurement.timestamp_ns <= self._last_zupt_timestamp_ns
        ):
            self._zupt_drop_count += 1
            self._last_timestamp_ns = self._last_zupt_timestamp_ns
            return self._build_estimate(
                imu_sample_rejected=False,
                zupt_sample_rejected=True,
                checkpoint_just_committed=False,
            )

        self._last_zupt_timestamp_ns = int(measurement.timestamp_ns)
        self._have_zupt_timestamp = True

        if measurement.stationary_flag is not True:
            self._zupt_drop_count += 1
            return self._build_estimate(
                imu_sample_rejected=False,
                zupt_sample_rejected=True,
                checkpoint_just_committed=False,
            )

        measurement_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            float(measurement.zero_velocity_variance_mps2),
        )
        innovation_variance_mps2: float = (
            self._forward_speed_variance_mps2 + measurement_variance_mps2
        )
        kalman_gain: float = (
            self._forward_speed_variance_mps2 / innovation_variance_mps2
        )
        self._forward_speed_mps += kalman_gain * (0.0 - self._forward_speed_mps)
        self._forward_speed_variance_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            (1.0 - kalman_gain) * self._forward_speed_variance_mps2,
        )
        return self._build_estimate(
            imu_sample_rejected=False,
            zupt_sample_rejected=False,
            checkpoint_just_committed=False,
        )

    def store_committed_forward_yaw(self, *, hostname: str) -> bool:
        """
        Persist the current committed forward yaw using the repo convention.
        """

        record: PersistenceRecord = PersistenceRecord(
            version=1,
            created_unix_ns=time.time_ns(),
            hostname=hostname,
            estimator="ahrs_forward_twist",
            valid=(self._committed_sample_count > 0),
            forward_yaw_rad=self._committed_forward_yaw_rad,
            forward_axis_xyz=(
                *_yaw_to_axis_xy(self._committed_forward_yaw_rad),
                0.0,
            ),
            fit_sample_count=self._committed_sample_count,
            checkpoint_count=self._checkpoint_commit_count,
        )
        try:
            self._persistence.store(record=record)
        except OSError as exc:
            self._persistence_failure_count += 1
            self._last_persistence_error = str(exc)
            return False

        self._persistence_success_count += 1
        self._last_persistence_error = ""
        return True

    def get_estimate(self) -> ForwardTwistEstimate:
        """
        Return the current forward-twist estimate.
        """

        return self._build_estimate(
            imu_sample_rejected=False,
            zupt_sample_rejected=False,
            checkpoint_just_committed=False,
        )

    def _update_learning(
        self,
        *,
        orientation_xyzw: tuple[float, float, float, float],
        linear_acceleration_mps2: tuple[float, float, float],
    ) -> bool:
        """
        Update candidate checkpoint evidence from straight-motion IMU samples.
        """

        _ = _yaw_from_quaternion_xyzw(orientation_xyzw)

        candidate_direction_xy: Optional[tuple[float, float]] = (
            _horizontal_direction_xy(
                linear_acceleration_mps2=linear_acceleration_mps2,
                min_norm_mps2=self._config.learning_accel_threshold_mps2,
            )
        )
        if candidate_direction_xy is None:
            return False

        if self._last_stationary_flag is True:
            return False

        if self._forward_sign_reference_xy is None:
            self._forward_sign_reference_xy = candidate_direction_xy

        aligned_direction_xy: tuple[float, float] = candidate_direction_xy
        if _dot2(candidate_direction_xy, self._forward_sign_reference_xy) < 0.0:
            aligned_direction_xy = (
                -candidate_direction_xy[0],
                -candidate_direction_xy[1],
            )

        self._uncommitted_direction_sum_xy = (
            self._uncommitted_direction_sum_xy[0] + aligned_direction_xy[0],
            self._uncommitted_direction_sum_xy[1] + aligned_direction_xy[1],
        )
        self._uncommitted_sample_count += 1
        self._fit_sample_count += 1

        candidate_sum_xy: tuple[float, float] = (
            self._committed_direction_sum_xy[0] + self._uncommitted_direction_sum_xy[0],
            self._committed_direction_sum_xy[1] + self._uncommitted_direction_sum_xy[1],
        )
        self._candidate_sample_count = (
            self._committed_sample_count + self._uncommitted_sample_count
        )
        if self._candidate_sample_count > 0:
            self._candidate_confidence = min(
                1.0,
                math.hypot(candidate_sum_xy[0], candidate_sum_xy[1])
                / float(self._candidate_sample_count),
            )
            if (
                math.hypot(candidate_sum_xy[0], candidate_sum_xy[1])
                >= MIN_DIRECTION_NORM
            ):
                self._candidate_forward_yaw_rad = math.atan2(
                    candidate_sum_xy[1], candidate_sum_xy[0]
                )

        return self._maybe_commit_checkpoint()

    def _maybe_commit_checkpoint(self) -> bool:
        """Commit the current candidate buffer when it is stable enough."""

        if self._candidate_sample_count < self._config.learning_min_samples:
            return False

        if self._uncommitted_sample_count < self._config.checkpoint_min_samples:
            return False

        if self._candidate_confidence < self._config.learning_min_confidence:
            return False

        if self._committed_sample_count > 0:
            yaw_delta_rad: float = abs(
                _wrap_angle_rad(
                    self._candidate_forward_yaw_rad - self._committed_forward_yaw_rad
                )
            )
            if yaw_delta_rad > self._config.checkpoint_max_candidate_delta_rad:
                return False

        self._committed_direction_sum_xy = (
            self._committed_direction_sum_xy[0] + self._uncommitted_direction_sum_xy[0],
            self._committed_direction_sum_xy[1] + self._uncommitted_direction_sum_xy[1],
        )
        self._committed_sample_count += self._uncommitted_sample_count
        self._committed_forward_yaw_rad = self._candidate_forward_yaw_rad
        self._checkpoint_commit_count += 1
        self._uncommitted_direction_sum_xy = (0.0, 0.0)
        self._uncommitted_sample_count = 0
        self._candidate_sample_count = self._committed_sample_count
        return True

    def _discard_uncommitted_learning(self) -> None:
        """Discard the current uncommitted learning buffer after a turn."""

        if self._uncommitted_sample_count <= 0:
            self._candidate_sample_count = self._committed_sample_count
            self._candidate_forward_yaw_rad = self._committed_forward_yaw_rad
            return

        self._checkpoint_discard_count += 1
        self._uncommitted_direction_sum_xy = (0.0, 0.0)
        self._uncommitted_sample_count = 0
        self._candidate_sample_count = self._committed_sample_count
        self._candidate_confidence = 1.0 if self._committed_sample_count > 0 else 0.0
        self._candidate_forward_yaw_rad = self._committed_forward_yaw_rad

    def _build_estimate(
        self,
        *,
        imu_sample_rejected: bool,
        zupt_sample_rejected: bool,
        checkpoint_just_committed: bool,
    ) -> ForwardTwistEstimate:
        """Build one immutable estimate snapshot."""

        forward_yaw_rad: float = (
            self._committed_forward_yaw_rad
            if self._committed_sample_count > 0
            else self._candidate_forward_yaw_rad
        )
        forward_axis_xyz: tuple[float, float, float] = (
            *_yaw_to_axis_xy(forward_yaw_rad),
            0.0,
        )
        forward_speed_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            self._forward_speed_variance_mps2,
        )
        return ForwardTwistEstimate(
            timestamp_ns=self._last_timestamp_ns,
            forward_speed_mps=self._forward_speed_mps,
            forward_speed_variance_mps2=forward_speed_variance_mps2,
            forward_speed_sigma_mps=math.sqrt(forward_speed_variance_mps2),
            forward_axis=ForwardAxisState(
                forward_yaw_rad=forward_yaw_rad,
                forward_axis_xyz=forward_axis_xyz,
                learned=(self._committed_sample_count > 0),
            ),
            learning_state=LearningState(
                candidate_forward_yaw_rad=self._candidate_forward_yaw_rad,
                committed_forward_yaw_rad=self._committed_forward_yaw_rad,
                candidate_sample_count=self._candidate_sample_count,
                committed_sample_count=self._committed_sample_count,
                uncommitted_sample_count=self._uncommitted_sample_count,
                candidate_confidence=self._candidate_confidence,
                forward_sign_locked=(self._forward_sign_reference_xy is not None),
                learning_gated_by_turn=self._last_turn_detected,
                checkpoint_commit_count=self._checkpoint_commit_count,
                checkpoint_discard_count=self._checkpoint_discard_count,
                checkpoint_just_committed=checkpoint_just_committed,
            ),
            turn_detected=self._last_turn_detected,
            imu_sample_rejected=imu_sample_rejected,
            zupt_sample_rejected=zupt_sample_rejected,
        )


def _horizontal_direction_xy(
    *,
    linear_acceleration_mps2: tuple[float, float, float],
    min_norm_mps2: float,
) -> Optional[tuple[float, float]]:
    """Return the normalized horizontal acceleration direction in `base_link`."""

    horizontal_norm_mps2: float = math.hypot(
        float(linear_acceleration_mps2[0]),
        float(linear_acceleration_mps2[1]),
    )
    if not math.isfinite(horizontal_norm_mps2) or horizontal_norm_mps2 < max(
        MIN_DIRECTION_NORM, min_norm_mps2
    ):
        return None

    return (
        float(linear_acceleration_mps2[0]) / horizontal_norm_mps2,
        float(linear_acceleration_mps2[1]) / horizontal_norm_mps2,
    )


def _yaw_from_quaternion_xyzw(
    quaternion_xyzw: tuple[float, float, float, float],
) -> float:
    """Return Z yaw in radians from one finite XYZW quaternion."""

    x_component: float = float(quaternion_xyzw[0])
    y_component: float = float(quaternion_xyzw[1])
    z_component: float = float(quaternion_xyzw[2])
    w_component: float = float(quaternion_xyzw[3])

    numerator: float = 2.0 * (w_component * z_component + x_component * y_component)
    denominator: float = 1.0 - 2.0 * (
        y_component * y_component + z_component * z_component
    )
    return math.atan2(numerator, denominator)


def _yaw_to_axis_xy(forward_yaw_rad: float) -> tuple[float, float]:
    """Return the horizontal forward-axis unit vector from one yaw."""

    return (
        math.cos(forward_yaw_rad),
        math.sin(forward_yaw_rad),
    )


def _quaternion_is_finite(
    quaternion_xyzw: tuple[float, float, float, float],
) -> bool:
    """Return true when all quaternion components are finite."""

    return all(math.isfinite(float(value)) for value in quaternion_xyzw)


def _dot2(lhs: tuple[float, float], rhs: tuple[float, float]) -> float:
    """Return the 2D dot product."""

    return lhs[0] * rhs[0] + lhs[1] * rhs[1]


def _wrap_angle_rad(angle_rad: float) -> float:
    """Wrap one angle to [-pi, pi)."""

    wrapped_angle_rad: float = math.fmod(float(angle_rad) + math.pi, 2.0 * math.pi)
    if wrapped_angle_rad < 0.0:
        wrapped_angle_rad += 2.0 * math.pi
    return wrapped_angle_rad - math.pi
