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
from pathlib import Path
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

# Units: samples
# Meaning: floor on first-commit evidence depth so the first committed fit is
# materially stronger than the relaxed ongoing-learning threshold.
FIRST_COMMIT_MIN_SAMPLES_FLOOR: int = 8

# Units: checkpoint windows
# Meaning: minimum number of checkpoint-sized evidence windows that must agree
# before the first fit becomes authoritative.
FIRST_COMMIT_MIN_CHECKPOINTS: int = 2

# Units: confidence
# Meaning: first committed fit confidence floor.
FIRST_COMMIT_MIN_CONFIDENCE: float = 0.88

# Units: residual
# Meaning: first committed fit residual ceiling.
FIRST_COMMIT_MAX_RESIDUAL: float = 0.12

# Units: rad
# Meaning: first committed fit yaw-uncertainty ceiling.
FIRST_COMMIT_MAX_UNCERTAINTY_RAD: float = 0.12

# Units: alignment
# Meaning: recent-window stability floor for the first committed fit.
FIRST_COMMIT_MIN_RECENT_STABILITY: float = 0.95

# Units: alignment
# Meaning: recent-window stability floor before a later candidate may replace
# the committed fit.
REPLACEMENT_MIN_RECENT_STABILITY: float = 0.82

# Units: score
# Meaning: minimum score improvement required for an unambiguous better-fit
# replacement.
BETTER_SCORE_MARGIN: float = 0.08

# Units: score
# Meaning: allowable score slack when a candidate wins via materially lower
# uncertainty or materially lower residual instead of raw score.
COMPARABLE_SCORE_TOLERANCE: float = 0.02

# Units: confidence
# Meaning: a better candidate may be slightly noisier than the current
# committed estimate, but not materially worse.
CONFIDENCE_TOLERANCE: float = 0.02

# Units: rad
# Meaning: a better candidate may have slightly larger uncertainty than the
# current committed estimate, but not by more than this tolerance.
UNCERTAINTY_TOLERANCE_RAD: float = 0.02

# Units: samples
# Meaning: sample-count improvement that by itself counts as meaningful once the
# main score gate is already passed.
MEANINGFUL_SAMPLE_DELTA: int = 2

# Units: checkpoint windows
# Meaning: checkpoint-depth improvement that counts as meaningful when deciding
# whether a candidate fit is genuinely better.
MEANINGFUL_CHECKPOINT_DELTA: int = 1

# Units: confidence
# Meaning: confidence improvement that counts as meaningful once the main score
# gate is already passed.
MEANINGFUL_CONFIDENCE_DELTA: float = 0.01

# Units: residual
# Meaning: residual improvement that counts as a materially better fit.
MEANINGFUL_RESIDUAL_DELTA: float = 0.03

# Units: rad
# Meaning: uncertainty improvement that counts as meaningful once the main
# score gate is already passed.
MEANINGFUL_UNCERTAINTY_DELTA_RAD: float = 0.002

# Units: samples
# Meaning: recent evidence window length used to ensure the candidate fit is
# stable over more than one brief transient.
RECENT_STABILITY_WINDOW_SIZE: int = 8


class ForwardTwistEstimator:
    """
    Estimate a learned forward axis plus signed scalar forward speed.

    The estimator continuously refines a committed forward axis during runtime.
    It may initialize that committed state from disk on startup. New candidate
    evidence only replaces the committed estimate when a deterministic quality
    rule says the candidate is genuinely better.
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
        self._committed_confidence: float = 0.0
        self._candidate_score: float = 0.0
        self._committed_score: float = 0.0
        self._candidate_residual: float = 1.0
        self._committed_residual: float = 1.0
        self._candidate_uncertainty_forward_yaw_rad: float = math.pi
        self._committed_uncertainty_forward_yaw_rad: float = math.pi
        self._candidate_beats_committed: bool = False
        self._candidate_recent_stability: float = 0.0
        self._checkpoint_commit_count: int = 0
        self._checkpoint_discard_count: int = 0
        self._fit_sample_count: int = 0

        self._committed_direction_sum_xy: tuple[float, float] = (0.0, 0.0)
        self._uncommitted_direction_sum_xy: tuple[float, float] = (0.0, 0.0)
        self._forward_sign_reference_xy: Optional[tuple[float, float]] = None
        self._recent_aligned_directions_xy: list[tuple[float, float]] = []

        self._forward_speed_mps: float = 0.0
        self._forward_speed_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            self._config.initial_forward_speed_sigma_mps**2,
        )

        self._last_turn_detected: bool = False
        self._last_stationary_flag: Optional[bool] = None
        self._last_stationary_flag_timestamp_ns: Optional[int] = None
        self._last_timestamp_ns: int = 0
        self._last_imu_timestamp_ns: int = 0
        self._last_zupt_timestamp_ns: int = 0
        self._have_imu_timestamp: bool = False
        self._have_zupt_timestamp: bool = False
        self._last_projected_forward_accel_mps2: float = 0.0
        self._have_projected_forward_accel: bool = False
        self._imu_drop_count: int = 0
        self._zupt_drop_count: int = 0
        self._zupt_applied_count: int = 0
        self._zupt_rejected_stale_count: int = 0
        self._zupt_rejected_motion_count: int = 0
        self._last_zupt_update_applied: bool = False
        self._last_zupt_rejected_stale: bool = False
        self._last_zupt_rejected_motion_contradiction: bool = False
        self._last_zupt_measurement_variance_used_mps2: Optional[float] = None
        self._last_zupt_kalman_gain: float = 0.0

        self._startup_loaded_from_persistence: bool = False
        self._persistence_load_valid: bool = False
        self._persistence_load_error: str = ""
        self._current_commit_from_persistence: bool = False
        self._last_commit_reason: str = "waiting_for_learning"
        self._last_persistence_reason: str = "not_written_yet"

        self._persistence_failure_count: int = 0
        self._persistence_success_count: int = 0
        self._last_persistence_error: str = ""

        self._load_committed_forward_yaw()

    @property
    def persistence_success_count(self) -> int:
        """Return the number of successful committed-estimate writes."""

        return self._persistence_success_count

    @property
    def persistence_failure_count(self) -> int:
        """Return the number of failed committed-estimate writes."""

        return self._persistence_failure_count

    @property
    def persistence_write_count(self) -> int:
        """Return the cumulative count of successful persistence writes."""

        return self._persistence_success_count

    @property
    def last_persistence_error(self) -> str:
        """Return the latest persistence failure text, if any."""

        return self._last_persistence_error

    @property
    def last_persistence_reason(self) -> str:
        """Return the latest persistence load/write reason string."""

        return self._last_persistence_reason

    @property
    def persistence_load_valid(self) -> bool:
        """Return true when startup load succeeded."""

        return self._persistence_load_valid

    @property
    def persistence_load_error(self) -> str:
        """Return the startup load failure reason, if any."""

        return self._persistence_load_error

    @property
    def startup_loaded_from_persistence(self) -> bool:
        """Return true when startup initialized from a persisted value."""

        return self._startup_loaded_from_persistence

    @property
    def current_commit_from_persistence(self) -> bool:
        """Return true when the active committed yaw still originates from disk."""

        return self._current_commit_from_persistence

    @property
    def persistence_path(self) -> Path:
        """Return the host-specific persistence path."""

        return self._persistence.path

    @property
    def last_commit_reason(self) -> str:
        """Return the latest commit or candidate-rejection reason."""

        return self._last_commit_reason

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
            self._last_projected_forward_accel_mps2 = projected_forward_accel_mps2
            self._have_projected_forward_accel = True
            self._forward_speed_mps += projected_forward_accel_mps2 * dt_sec
            self._forward_speed_variance_mps2 = max(
                self._config.min_forward_speed_variance_mps2,
                self._forward_speed_variance_mps2
                + dt_sec
                * dt_sec
                * self._config.forward_accel_process_sigma_mps2
                * self._config.forward_accel_process_sigma_mps2,
            )

        return self._build_estimate(
            imu_sample_rejected=False,
            zupt_sample_rejected=False,
            checkpoint_just_committed=checkpoint_just_committed,
        )

    def update_zupt(self, *, measurement: ZuptMeasurement) -> ForwardTwistEstimate:
        """
        Apply a scalar zero-velocity update when the paired ZUPT flag is true.
        """

        self._last_stationary_flag = measurement.stationary_flag
        self._last_stationary_flag_timestamp_ns = (
            measurement.stationary_flag_timestamp_ns
        )
        self._last_timestamp_ns = int(measurement.timestamp_ns)
        self._last_zupt_update_applied = False
        self._last_zupt_rejected_stale = False
        self._last_zupt_rejected_motion_contradiction = False
        self._last_zupt_measurement_variance_used_mps2 = None
        self._last_zupt_kalman_gain = 0.0

        if (
            self._have_zupt_timestamp
            and measurement.timestamp_ns <= self._last_zupt_timestamp_ns
        ):
            self._zupt_drop_count += 1
            self._zupt_rejected_stale_count += 1
            self._last_zupt_rejected_stale = True
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

        if self._have_imu_timestamp:
            if (
                self._zupt_is_stale(
                    timestamp_ns=measurement.timestamp_ns,
                    reference_timestamp_ns=self._last_imu_timestamp_ns,
                )
                or self._stationary_flag_is_stale()
            ):
                self._zupt_drop_count += 1
                self._zupt_rejected_stale_count += 1
                self._last_zupt_rejected_stale = True
                return self._build_estimate(
                    imu_sample_rejected=False,
                    zupt_sample_rejected=True,
                    checkpoint_just_committed=False,
                )

        if self._motion_contradicts_stationarity():
            self._zupt_drop_count += 1
            self._zupt_rejected_motion_count += 1
            self._last_zupt_rejected_motion_contradiction = True
            return self._build_estimate(
                imu_sample_rejected=False,
                zupt_sample_rejected=True,
                checkpoint_just_committed=False,
            )

        measurement_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            float(measurement.zero_velocity_variance_mps2),
        )
        self._last_zupt_measurement_variance_used_mps2 = measurement_variance_mps2
        innovation_variance_mps2: float = (
            self._forward_speed_variance_mps2 + measurement_variance_mps2
        )
        kalman_gain: float = (
            self._forward_speed_variance_mps2 / innovation_variance_mps2
        )
        self._last_zupt_kalman_gain = kalman_gain
        self._forward_speed_mps += kalman_gain * (0.0 - self._forward_speed_mps)
        self._forward_speed_variance_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            (1.0 - kalman_gain) * self._forward_speed_variance_mps2,
        )
        self._zupt_applied_count += 1
        self._last_zupt_update_applied = True
        return self._build_estimate(
            imu_sample_rejected=False,
            zupt_sample_rejected=False,
            checkpoint_just_committed=False,
        )

    def get_estimate(self) -> ForwardTwistEstimate:
        """
        Return the current forward-twist estimate.
        """

        return self._build_estimate(
            imu_sample_rejected=False,
            zupt_sample_rejected=False,
            checkpoint_just_committed=False,
        )

    def store_committed_forward_yaw(self, *, hostname: str, reason: str) -> bool:
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
            confidence=self._committed_confidence,
            score=self._committed_score,
            residual=self._committed_residual,
            uncertainty_forward_yaw_rad=self._committed_uncertainty_forward_yaw_rad,
            loaded_startup_capable=True,
            last_update_reason=reason,
        )
        try:
            self._persistence.store(record=record)
        except OSError as exc:
            self._persistence_failure_count += 1
            self._last_persistence_error = str(exc)
            self._last_persistence_reason = f"write_failed:{exc}"
            return False

        self._persistence_success_count += 1
        self._last_persistence_error = ""
        self._last_persistence_reason = reason
        return True

    def _load_committed_forward_yaw(self) -> None:
        """Load the persisted committed forward yaw for direct startup use."""

        self._persistence_load_valid = False
        self._persistence_load_error = ""
        self._startup_loaded_from_persistence = False
        self._current_commit_from_persistence = False

        try:
            record: PersistenceRecord = self._persistence.load()
        except FileNotFoundError:
            self._persistence_load_error = "missing_persistence_file"
            self._last_persistence_reason = "startup_fallback_missing_file"
            return
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self._persistence_load_error = str(exc)
            self._last_persistence_reason = "startup_fallback_invalid_file"
            return

        self._committed_forward_yaw_rad = _wrap_angle_rad(record.forward_yaw_rad)
        self._candidate_forward_yaw_rad = self._committed_forward_yaw_rad
        self._committed_sample_count = max(0, int(record.fit_sample_count))
        self._candidate_sample_count = self._committed_sample_count
        self._committed_confidence = max(0.0, min(1.0, float(record.confidence)))
        self._candidate_confidence = self._committed_confidence
        self._committed_residual = max(0.0, float(record.residual))
        self._candidate_residual = self._committed_residual
        self._committed_uncertainty_forward_yaw_rad = max(
            0.0,
            float(record.uncertainty_forward_yaw_rad),
        )
        self._candidate_uncertainty_forward_yaw_rad = (
            self._committed_uncertainty_forward_yaw_rad
        )
        committed_checkpoint_count: int = max(0, int(record.checkpoint_count))
        self._committed_score = _compute_score(
            sample_count=self._committed_sample_count,
            checkpoint_count=committed_checkpoint_count,
            confidence=self._committed_confidence,
            residual=self._committed_residual,
            uncertainty_forward_yaw_rad=(self._committed_uncertainty_forward_yaw_rad),
        )
        self._candidate_score = self._committed_score
        self._checkpoint_commit_count = committed_checkpoint_count
        self._fit_sample_count = self._committed_sample_count
        self._startup_loaded_from_persistence = True
        self._persistence_load_valid = True
        self._current_commit_from_persistence = True
        self._last_commit_reason = "startup_loaded_persistence"
        self._last_persistence_reason = "startup_loaded_persistence"
        axis_xy: tuple[float, float] = _yaw_to_axis_xy(self._committed_forward_yaw_rad)
        self._forward_sign_reference_xy = axis_xy
        evidence_norm: float = self._committed_confidence * float(
            self._committed_sample_count
        )
        self._committed_direction_sum_xy = (
            evidence_norm * axis_xy[0],
            evidence_norm * axis_xy[1],
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
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_low_excitation"
            return False

        if self._last_stationary_flag is True:
            if self._stationary_flag_is_stale():
                self._last_stationary_flag = False
            else:
                self._candidate_beats_committed = False
                self._last_commit_reason = "candidate_rejected_stationary"
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
        self._append_recent_direction(aligned_direction_xy)
        self._uncommitted_sample_count += 1
        self._fit_sample_count += 1
        self._update_candidate_metrics()
        return self._maybe_commit_checkpoint()

    def _update_candidate_metrics(self) -> None:
        """Update candidate yaw and quality metrics from accumulated evidence."""

        candidate_sum_xy: tuple[float, float] = (
            self._committed_direction_sum_xy[0] + self._uncommitted_direction_sum_xy[0],
            self._committed_direction_sum_xy[1] + self._uncommitted_direction_sum_xy[1],
        )
        self._candidate_sample_count = (
            self._committed_sample_count + self._uncommitted_sample_count
        )
        candidate_norm: float = math.hypot(candidate_sum_xy[0], candidate_sum_xy[1])
        if self._candidate_sample_count > 0:
            self._candidate_confidence = min(
                1.0,
                candidate_norm / float(self._candidate_sample_count),
            )
        else:
            self._candidate_confidence = 0.0

        if candidate_norm >= MIN_DIRECTION_NORM:
            self._candidate_forward_yaw_rad = math.atan2(
                candidate_sum_xy[1],
                candidate_sum_xy[0],
            )

        self._candidate_residual = max(0.0, 1.0 - self._candidate_confidence)
        self._candidate_uncertainty_forward_yaw_rad = _estimate_uncertainty_rad(
            sample_count=self._candidate_sample_count,
            residual=self._candidate_residual,
        )
        self._candidate_recent_stability = _compute_recent_stability(
            directions_xy=self._recent_aligned_directions_xy
        )
        self._candidate_score = _compute_score(
            sample_count=self._candidate_sample_count,
            checkpoint_count=self._candidate_checkpoint_count(),
            confidence=self._candidate_confidence,
            residual=self._candidate_residual,
            uncertainty_forward_yaw_rad=self._candidate_uncertainty_forward_yaw_rad,
        )

    def _maybe_commit_checkpoint(self) -> bool:
        """
        Commit the current candidate buffer when it is genuinely better.

        Better-value rule:
        1. The first committed fit uses a stricter gate than steady-state
           refinement. It must have deeper evidence, lower residual, lower yaw
           uncertainty, and strong recent-window stability.
        2. Later candidates may replace the committed fit when one of three
           deterministic wins applies:
           a. materially better total score
           b. materially lower uncertainty with comparable score
           c. materially lower residual with comparable confidence and score
        3. Replacement candidates must also be stable over the recent evidence
           window and must not be materially worse in confidence.
        """

        if self._candidate_sample_count < self._config.learning_min_samples:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_min_samples"
            return False

        if self._uncommitted_sample_count < self._config.checkpoint_min_samples:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_checkpoint_window"
            return False

        if self._candidate_confidence < self._config.learning_min_confidence:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_confidence"
            return False

        if self._committed_sample_count <= 0:
            return self._maybe_accept_first_commit()

        yaw_delta_rad: float = abs(
            _wrap_angle_rad(
                self._candidate_forward_yaw_rad - self._committed_forward_yaw_rad
            )
        )
        if yaw_delta_rad > self._config.checkpoint_max_candidate_delta_rad:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_large_delta"
            return False

        score_improvement: float = self._candidate_score - self._committed_score
        confidence_improvement: float = (
            self._candidate_confidence - self._committed_confidence
        )
        residual_improvement: float = (
            self._committed_residual - self._candidate_residual
        )
        uncertainty_improvement_rad: float = (
            self._committed_uncertainty_forward_yaw_rad
            - self._candidate_uncertainty_forward_yaw_rad
        )
        sample_improvement: int = (
            self._candidate_sample_count - self._committed_sample_count
        )
        checkpoint_improvement: int = (
            self._candidate_checkpoint_count() - self._checkpoint_commit_count
        )

        meaningful_improvement: bool = (
            sample_improvement >= MEANINGFUL_SAMPLE_DELTA
            or checkpoint_improvement >= MEANINGFUL_CHECKPOINT_DELTA
            or confidence_improvement >= MEANINGFUL_CONFIDENCE_DELTA
            or residual_improvement >= MEANINGFUL_RESIDUAL_DELTA
            or uncertainty_improvement_rad >= MEANINGFUL_UNCERTAINTY_DELTA_RAD
        )
        wins_by_score: bool = score_improvement >= BETTER_SCORE_MARGIN
        wins_by_uncertainty: bool = (
            uncertainty_improvement_rad >= MEANINGFUL_UNCERTAINTY_DELTA_RAD
            and score_improvement >= -COMPARABLE_SCORE_TOLERANCE
        )
        wins_by_residual: bool = (
            residual_improvement >= MEANINGFUL_RESIDUAL_DELTA
            and score_improvement >= -COMPARABLE_SCORE_TOLERANCE
            and self._candidate_confidence
            >= self._committed_confidence - CONFIDENCE_TOLERANCE
        )
        better_enough: bool = (
            (wins_by_score or wins_by_uncertainty or wins_by_residual)
            and self._candidate_confidence
            >= self._committed_confidence - CONFIDENCE_TOLERANCE
            and self._candidate_uncertainty_forward_yaw_rad
            <= self._committed_uncertainty_forward_yaw_rad + UNCERTAINTY_TOLERANCE_RAD
            and self._candidate_recent_stability >= REPLACEMENT_MIN_RECENT_STABILITY
            and meaningful_improvement
        )
        self._candidate_beats_committed = better_enough
        if not better_enough:
            if self._candidate_recent_stability < REPLACEMENT_MIN_RECENT_STABILITY:
                self._last_commit_reason = "candidate_rejected_recent_instability"
            else:
                self._last_commit_reason = "candidate_kept_as_weaker_than_committed"
            return False

        if wins_by_score:
            return self._accept_candidate(reason="candidate_score_beat_committed")
        if wins_by_uncertainty:
            return self._accept_candidate(reason="candidate_uncertainty_beat_committed")

        return self._accept_candidate(reason="candidate_residual_beat_committed")

    def _maybe_accept_first_commit(self) -> bool:
        """Return true when the first committed fit is strong enough."""

        first_commit_min_samples: int = max(
            FIRST_COMMIT_MIN_SAMPLES_FLOOR,
            2 * self._config.learning_min_samples,
        )
        if self._candidate_sample_count < first_commit_min_samples:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_first_commit_samples"
            return False

        if self._candidate_checkpoint_count() < FIRST_COMMIT_MIN_CHECKPOINTS:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_first_commit_checkpoints"
            return False

        if self._candidate_confidence < max(
            FIRST_COMMIT_MIN_CONFIDENCE,
            self._config.learning_min_confidence,
        ):
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_first_commit_confidence"
            return False

        if self._candidate_residual > FIRST_COMMIT_MAX_RESIDUAL:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_first_commit_residual"
            return False

        if (
            self._candidate_uncertainty_forward_yaw_rad
            > FIRST_COMMIT_MAX_UNCERTAINTY_RAD
        ):
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_first_commit_uncertainty"
            return False

        if self._candidate_recent_stability < FIRST_COMMIT_MIN_RECENT_STABILITY:
            self._candidate_beats_committed = False
            self._last_commit_reason = "candidate_rejected_first_commit_stability"
            return False

        self._candidate_beats_committed = True
        return self._accept_candidate(reason="first_committed_estimate")

    def _accept_candidate(self, *, reason: str) -> bool:
        """Accept the current candidate as the new committed estimate."""

        self._committed_direction_sum_xy = (
            self._committed_direction_sum_xy[0] + self._uncommitted_direction_sum_xy[0],
            self._committed_direction_sum_xy[1] + self._uncommitted_direction_sum_xy[1],
        )
        self._committed_sample_count = self._candidate_sample_count
        self._committed_forward_yaw_rad = self._candidate_forward_yaw_rad
        self._committed_confidence = self._candidate_confidence
        self._committed_score = self._candidate_score
        self._committed_residual = self._candidate_residual
        self._committed_uncertainty_forward_yaw_rad = (
            self._candidate_uncertainty_forward_yaw_rad
        )
        self._checkpoint_commit_count += 1
        self._uncommitted_direction_sum_xy = (0.0, 0.0)
        self._uncommitted_sample_count = 0
        self._candidate_sample_count = self._committed_sample_count
        self._current_commit_from_persistence = False
        self._last_commit_reason = reason
        self._candidate_beats_committed = True

        if self._config.persistence_write_on_checkpoint:
            self.store_committed_forward_yaw(
                hostname=self._persistence.hostname,
                reason=reason,
            )
        return True

    def _discard_uncommitted_learning(self) -> None:
        """Discard the current uncommitted learning buffer after a turn."""

        if self._uncommitted_sample_count <= 0:
            self._candidate_sample_count = self._committed_sample_count
            self._candidate_forward_yaw_rad = self._committed_forward_yaw_rad
            self._candidate_confidence = self._committed_confidence
            self._candidate_score = self._committed_score
            self._candidate_residual = self._committed_residual
            self._candidate_uncertainty_forward_yaw_rad = (
                self._committed_uncertainty_forward_yaw_rad
            )
            self._candidate_recent_stability = 0.0
            self._candidate_beats_committed = False
            self._last_commit_reason = "no_uncommitted_evidence_to_discard"
            return

        self._checkpoint_discard_count += 1
        self._uncommitted_direction_sum_xy = (0.0, 0.0)
        self._uncommitted_sample_count = 0
        self._recent_aligned_directions_xy.clear()
        self._candidate_sample_count = self._committed_sample_count
        self._candidate_forward_yaw_rad = self._committed_forward_yaw_rad
        self._candidate_confidence = self._committed_confidence
        self._candidate_score = self._committed_score
        self._candidate_residual = self._committed_residual
        self._candidate_uncertainty_forward_yaw_rad = (
            self._committed_uncertainty_forward_yaw_rad
        )
        self._candidate_recent_stability = 0.0
        self._candidate_beats_committed = False
        self._last_commit_reason = "discarded_uncommitted_due_to_turn"

    def _append_recent_direction(
        self,
        aligned_direction_xy: tuple[float, float],
    ) -> None:
        """Append one aligned direction to the recent stability window."""

        self._recent_aligned_directions_xy.append(aligned_direction_xy)
        if len(self._recent_aligned_directions_xy) > RECENT_STABILITY_WINDOW_SIZE:
            self._recent_aligned_directions_xy.pop(0)

    def _candidate_checkpoint_count(self) -> int:
        """Return the current candidate checkpoint-depth proxy."""

        return self._checkpoint_commit_count + (
            self._uncommitted_sample_count // self._config.checkpoint_min_samples
        )

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
                committed_confidence=self._committed_confidence,
                committed_score=self._committed_score,
                candidate_score=self._candidate_score,
                committed_residual=self._committed_residual,
                candidate_residual=self._candidate_residual,
                committed_uncertainty_forward_yaw_rad=(
                    self._committed_uncertainty_forward_yaw_rad
                ),
                candidate_uncertainty_forward_yaw_rad=(
                    self._candidate_uncertainty_forward_yaw_rad
                ),
                candidate_beats_committed=self._candidate_beats_committed,
                last_commit_reason=self._last_commit_reason,
                committed_source=(
                    "persistence"
                    if self._current_commit_from_persistence
                    else "learning"
                ),
            ),
            turn_detected=self._last_turn_detected,
            latest_zupt_flag_age_sec=self._latest_zupt_flag_age_sec(),
            latest_zupt_age_sec=self._latest_zupt_age_sec(),
            zupt_update_applied=self._last_zupt_update_applied,
            zupt_rejected_stale=self._last_zupt_rejected_stale,
            zupt_rejected_motion_contradiction=(
                self._last_zupt_rejected_motion_contradiction
            ),
            zupt_measurement_variance_used_mps2=(
                self._last_zupt_measurement_variance_used_mps2
            ),
            zupt_kalman_gain=self._last_zupt_kalman_gain,
            zupt_applied_count=self._zupt_applied_count,
            zupt_rejected_stale_count=self._zupt_rejected_stale_count,
            zupt_rejected_motion_count=self._zupt_rejected_motion_count,
            startup_loaded_from_persistence=self._startup_loaded_from_persistence,
            persistence_load_path=str(self._persistence.path),
            persistence_load_valid=self._persistence_load_valid,
            persistence_load_error=self._persistence_load_error,
            current_commit_from_persistence=self._current_commit_from_persistence,
            persistence_write_count=self._persistence_success_count,
            last_persistence_reason=self._last_persistence_reason,
            imu_sample_rejected=imu_sample_rejected,
            zupt_sample_rejected=zupt_sample_rejected,
        )

    def _zupt_is_stale(self, *, timestamp_ns: int, reference_timestamp_ns: int) -> bool:
        """Return true when one timestamp is too old relative to another."""

        age_sec: float = max(
            0.0,
            float(reference_timestamp_ns - timestamp_ns) * 1.0e-9,
        )
        return age_sec > self._config.zupt_freshness_window_sec

    def _stationary_flag_is_stale(self) -> bool:
        """Return true when the latest paired stationary flag is too old."""

        if (
            self._last_stationary_flag_timestamp_ns is None
            or not self._have_imu_timestamp
        ):
            return True

        return self._zupt_is_stale(
            timestamp_ns=self._last_stationary_flag_timestamp_ns,
            reference_timestamp_ns=self._last_imu_timestamp_ns,
        )

    def _motion_contradicts_stationarity(self) -> bool:
        """Return true when recent IMU motion strongly contradicts stationarity."""

        if not self._have_projected_forward_accel:
            return False

        if not self._have_imu_timestamp or not self._have_zupt_timestamp:
            return False

        if self._zupt_is_stale(
            timestamp_ns=self._last_zupt_timestamp_ns,
            reference_timestamp_ns=self._last_imu_timestamp_ns,
        ):
            return False

        projected_accel_mps2: float = abs(self._last_projected_forward_accel_mps2)
        if projected_accel_mps2 >= self._config.zupt_motion_reject_accel_threshold_mps2:
            return True

        return (
            projected_accel_mps2
            >= 0.5 * self._config.zupt_motion_reject_accel_threshold_mps2
            and abs(self._forward_speed_mps)
            >= self._config.zupt_motion_reject_speed_threshold_mps
        )

    def _latest_zupt_flag_age_sec(self) -> Optional[float]:
        """Return the age of the latest `zupt_flag` relative to the newest IMU."""

        if (
            self._last_stationary_flag_timestamp_ns is None
            or not self._have_imu_timestamp
        ):
            return None

        return max(
            0.0,
            float(self._last_imu_timestamp_ns - self._last_stationary_flag_timestamp_ns)
            * 1.0e-9,
        )

    def _latest_zupt_age_sec(self) -> Optional[float]:
        """Return the age of the latest `zupt` relative to the newest IMU."""

        if not self._have_zupt_timestamp or not self._have_imu_timestamp:
            return None

        return max(
            0.0,
            float(self._last_imu_timestamp_ns - self._last_zupt_timestamp_ns) * 1.0e-9,
        )


def _compute_score(
    *,
    sample_count: int,
    checkpoint_count: int,
    confidence: float,
    residual: float,
    uncertainty_forward_yaw_rad: float,
) -> float:
    """Return the deterministic better-value score."""

    return (
        3.0 * max(0.0, confidence)
        + 0.35 * math.log1p(max(0, sample_count))
        + 0.25 * math.log1p(max(0, checkpoint_count))
        - 1.75 * max(0.0, residual)
        - 1.25 * max(0.0, uncertainty_forward_yaw_rad)
    )


def _estimate_uncertainty_rad(*, sample_count: int, residual: float) -> float:
    """Return a simple yaw-uncertainty proxy from residual and sample count."""

    sample_term: float = 1.0 / math.sqrt(max(1, sample_count))
    return max(0.01, math.sqrt(max(0.0, residual)) * sample_term)


def _compute_recent_stability(
    *,
    directions_xy: list[tuple[float, float]],
) -> float:
    """Return sign-aligned stability of the recent direction window."""

    if not directions_xy:
        return 0.0

    sum_x: float = 0.0
    sum_y: float = 0.0
    for direction_xy in directions_xy:
        sum_x += direction_xy[0]
        sum_y += direction_xy[1]

    return min(1.0, math.hypot(sum_x, sum_y) / float(len(directions_xy)))


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
