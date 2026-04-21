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
from collections import deque
from dataclasses import dataclass
from typing import Optional

from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_to_rotation_matrix
from oasis_control.localization.common.algebra.quat import rotate_vector
from oasis_control.localization.common.algebra.quat import transpose_matrix
from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.localization.speedometer.contracts import ForwardTwistEstimate
from oasis_control.localization.speedometer.contracts import ZuptMeasurement
from oasis_control.localization.speedometer.forward_twist_persistence import (
    PersistedForwardYawMetadata,
)
from oasis_control.localization.speedometer.forward_twist_persistence import (
    load_persisted_forward_yaw,
)
from oasis_control.localization.speedometer.forward_twist_persistence import (
    write_persisted_forward_yaw,
)


@dataclass(frozen=True)
class _AxisEvidenceSample:
    flat_accel_x_mps2: float
    flat_accel_y_mps2: float


@dataclass(frozen=True)
class _CandidateAxisState:
    forward_yaw_rad: float
    confidence: float
    residual_ratio: float
    score: float
    sample_count: int


class ForwardTwistEstimator:
    """
    Yaw-only forward speedometer on a gravity-leveled flat surface.

    The estimator learns only a flat-surface rail axis. It does not treat one
    instantaneous acceleration vector as a signed motion-direction estimate and
    it never learns a free 3D axis in `base_link`.
    """

    def __init__(
        self,
        *,
        config: ForwardTwistConfig,
    ) -> None:
        """Initialize the yaw-only forward-twist estimator."""

        self._config: ForwardTwistConfig = config

        self._forward_speed_mps: float = 0.0
        self._forward_accel_bias_mps2: float = 0.0
        self._var_forward_speed_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            1.0,
        )
        self._cov_forward_speed_bias_mps3: float = 0.0
        self._var_forward_accel_bias_mps2_2: float = max(
            self._config.min_forward_accel_bias_variance_mps2_2,
            1.0,
        )
        self._committed_forward_yaw_rad: float = 0.0
        self._var_committed_forward_yaw_rad2: float = max(
            self._config.min_forward_yaw_variance_rad2,
            math.pi**2,
        )
        self._has_committed_forward_yaw: bool = False
        self._candidate_forward_yaw_rad: Optional[float] = None
        self._candidate_confidence: Optional[float] = None
        self._candidate_residual_ratio: Optional[float] = None
        self._candidate_score: Optional[float] = None
        self._yaw_rate_rads: float = 0.0
        self._var_yaw_rate_rads2: float = max(
            self._config.min_yaw_rate_variance_rads2,
            self._config.default_yaw_rate_variance_rads2,
        )
        self._turn_detected: bool = False
        self._last_timestamp_ns: Optional[int] = None
        self._last_forward_acceleration_timestamp_ns: Optional[int] = None
        self._last_forward_acceleration_mps2: Optional[float] = None
        self._loaded_from_persistence: bool = False
        self._learning_enabled: bool = True

        self._accepted_learning_sample_count: int = 0
        self._discarded_uncommitted_sample_count: int = 0
        self._checkpoint_commit_count: int = 0
        self._checkpoint_discard_count: int = 0

        self._evidence_window: deque[_AxisEvidenceSample] = deque()
        self._window_scatter_xx: float = 0.0
        self._window_scatter_xy: float = 0.0
        self._window_scatter_yy: float = 0.0

        self._maybe_load_persisted_forward_yaw()

    def update_imu(
        self,
        *,
        timestamp_ns: int,
        orientation_xyzw: tuple[float, float, float, float],
        angular_velocity_rads: tuple[float, float, float],
        linear_acceleration_mps2: tuple[float, float, float],
    ) -> ForwardTwistEstimate:
        """Update runtime state from one AHRS IMU sample."""

        measurement_timestamp_ns: int = int(timestamp_ns)
        if (
            self._last_timestamp_ns is not None
            and measurement_timestamp_ns <= self._last_timestamp_ns
        ):
            return self.get_estimate()

        normalized_orientation_xyzw: Optional[tuple[float, float, float, float]] = (
            normalize_quaternion_xyzw(orientation_xyzw)
        )
        if normalized_orientation_xyzw is None:
            if self._last_timestamp_ns is None:
                self._last_timestamp_ns = measurement_timestamp_ns
            return self.get_estimate()

        rotation_world_to_base = quaternion_to_rotation_matrix(
            normalized_orientation_xyzw
        )
        rotation_base_to_world = transpose_matrix(rotation_world_to_base)

        flat_linear_acceleration_mps2: tuple[float, float, float] = (
            _project_body_vector_onto_world_flat_surface(
                vector_body=linear_acceleration_mps2,
                rotation_world_to_base=rotation_world_to_base,
                rotation_base_to_world=rotation_base_to_world,
            )
        )

        if self._last_timestamp_ns is not None:
            dt_sec: float = (
                float(measurement_timestamp_ns - self._last_timestamp_ns) * 1.0e-9
            )
            active_forward_yaw_rad: float = self._active_forward_yaw_rad()
            forward_acceleration_mps2: float = _dot(
                _forward_axis_flat(active_forward_yaw_rad),
                flat_linear_acceleration_mps2,
            )
            corrected_forward_acceleration_mps2: float = (
                forward_acceleration_mps2 - self._forward_accel_bias_mps2
            )
            self._forward_speed_mps += dt_sec * corrected_forward_acceleration_mps2
            self._propagate_speed_and_bias_covariance(dt_sec=dt_sec)

            self._last_forward_acceleration_timestamp_ns = measurement_timestamp_ns
            self._last_forward_acceleration_mps2 = forward_acceleration_mps2
        else:
            active_forward_yaw_rad = self._active_forward_yaw_rad()
            self._last_forward_acceleration_timestamp_ns = measurement_timestamp_ns
            self._last_forward_acceleration_mps2 = _dot(
                _forward_axis_flat(active_forward_yaw_rad),
                flat_linear_acceleration_mps2,
            )

        self._last_timestamp_ns = measurement_timestamp_ns
        self._yaw_rate_rads = float(angular_velocity_rads[2])
        self._var_yaw_rate_rads2 = max(
            self._config.min_yaw_rate_variance_rads2,
            self._config.default_yaw_rate_variance_rads2,
        )

        self._update_forward_yaw_learning(
            timestamp_ns=measurement_timestamp_ns,
            yaw_rate_rads=self._yaw_rate_rads,
            flat_linear_acceleration_mps2=flat_linear_acceleration_mps2,
        )

        return self.get_estimate()

    def update_zupt(self, *, measurement: ZuptMeasurement) -> ForwardTwistEstimate:
        """Apply one scalar zero-speed correction from a stationary-twist ZUPT."""

        measurement_timestamp_ns: int = int(measurement.timestamp_ns)
        if (
            self._last_timestamp_ns is not None
            and measurement_timestamp_ns < self._last_timestamp_ns
        ):
            return self.get_estimate()

        self._last_timestamp_ns = measurement_timestamp_ns
        if measurement.stationary_flag is not True:
            return self.get_estimate()

        measurement_variance_mps2: float = max(
            self._config.min_forward_speed_variance_mps2,
            float(measurement.zero_velocity_variance_mps2),
        )
        self._apply_speed_measurement_update(
            measurement_value_mps=0.0,
            measurement_variance_mps2=measurement_variance_mps2,
        )

        self._apply_stationary_bias_update(
            measurement_timestamp_ns=measurement_timestamp_ns
        )
        return self.get_estimate()

    def get_estimate(self) -> ForwardTwistEstimate:
        """Return the current runtime estimate."""

        active_forward_yaw_rad: float = self._active_forward_yaw_rad()

        return ForwardTwistEstimate(
            timestamp_ns=(
                0 if self._last_timestamp_ns is None else self._last_timestamp_ns
            ),
            forward_yaw_rad=active_forward_yaw_rad,
            committed_forward_yaw_rad=self._committed_forward_yaw_rad,
            committed_forward_yaw_variance_rad2=self._var_committed_forward_yaw_rad2,
            candidate_forward_yaw_rad=self._candidate_forward_yaw_rad,
            candidate_confidence=self._candidate_confidence,
            candidate_residual_ratio=self._candidate_residual_ratio,
            forward_speed_mps=self._forward_speed_mps,
            forward_speed_variance_mps2=self._var_forward_speed_mps2,
            forward_accel_bias_mps2=self._forward_accel_bias_mps2,
            forward_accel_bias_variance_mps2_2=self._var_forward_accel_bias_mps2_2,
            yaw_rate_rads=self._yaw_rate_rads,
            yaw_rate_variance_rads2=self._var_yaw_rate_rads2,
            accepted_learning_sample_count=self._accepted_learning_sample_count,
            discarded_uncommitted_sample_count=(
                self._discarded_uncommitted_sample_count
            ),
            checkpoint_commit_count=self._checkpoint_commit_count,
            checkpoint_discard_count=self._checkpoint_discard_count,
            turn_detected=self._turn_detected,
            loaded_from_persistence=self._loaded_from_persistence,
            learning_enabled=self._learning_enabled,
        )

    def _maybe_load_persisted_forward_yaw(self) -> None:
        persistence_path: Optional[str] = self._config.persistence_path
        if not persistence_path:
            return

        persisted_forward_yaw = load_persisted_forward_yaw(persistence_path)
        if persisted_forward_yaw is None:
            return

        self._committed_forward_yaw_rad = _normalize_angle(
            float(persisted_forward_yaw.forward_yaw_rad)
        )
        self._var_committed_forward_yaw_rad2 = max(
            self._config.min_forward_yaw_variance_rad2,
            persisted_forward_yaw.uncertainty_forward_yaw_rad
            * persisted_forward_yaw.uncertainty_forward_yaw_rad,
        )
        self._has_committed_forward_yaw = True
        self._loaded_from_persistence = True
        self._learning_enabled = False
        self._clear_candidate_evidence()

    def _update_forward_yaw_learning(
        self,
        *,
        timestamp_ns: int,
        yaw_rate_rads: float,
        flat_linear_acceleration_mps2: tuple[float, float, float],
    ) -> None:
        if not self._learning_enabled:
            self._turn_detected = False
            return

        flat_accel_x_mps2: float = float(flat_linear_acceleration_mps2[0])
        flat_accel_y_mps2: float = float(flat_linear_acceleration_mps2[1])
        flat_accel_norm_mps2: float = math.hypot(
            flat_accel_x_mps2,
            flat_accel_y_mps2,
        )

        if flat_accel_norm_mps2 >= self._config.yaw_learning_accel_threshold_mps2:
            self._append_evidence_sample(
                _AxisEvidenceSample(
                    flat_accel_x_mps2=flat_accel_x_mps2,
                    flat_accel_y_mps2=flat_accel_y_mps2,
                )
            )
            self._accepted_learning_sample_count += 1

        candidate_state: Optional[_CandidateAxisState] = self._compute_candidate_axis()
        self._update_candidate_fields(candidate_state)

        turn_detected: bool = (
            abs(yaw_rate_rads) > self._config.yaw_learning_max_yaw_rate_rads
        )
        if (
            not turn_detected
            and candidate_state is not None
            and candidate_state.sample_count >= self._config.yaw_learning_min_samples
            and candidate_state.residual_ratio
            > self._config.yaw_learning_max_residual_ratio
        ):
            turn_detected = True

        self._turn_detected = turn_detected
        if turn_detected:
            self._discard_uncommitted_candidate()
            return

        if candidate_state is None:
            return

        if candidate_state.sample_count < self._config.yaw_learning_min_samples:
            return

        if candidate_state.confidence < self._config.yaw_learning_min_confidence:
            return

        if (
            candidate_state.residual_ratio
            > self._config.yaw_learning_max_residual_ratio
        ):
            return

        self._commit_candidate(
            timestamp_ns=timestamp_ns,
            candidate_state=candidate_state,
        )

    def _propagate_speed_and_bias_covariance(self, *, dt_sec: float) -> None:
        previous_var_forward_speed_mps2: float = self._var_forward_speed_mps2
        previous_cov_speed_bias_mps3: float = self._cov_forward_speed_bias_mps3
        previous_var_forward_accel_bias_mps2_2: float = (
            self._var_forward_accel_bias_mps2_2
        )

        # This is the instantaneous acceleration noise that drives the speed
        # propagation step. It is sanitized as a nonnegative process term and
        # is separate from any state-variance minimum.
        accel_drive_process_variance_mps2_2: float = _sanitize_process_variance(
            self._config.forward_accel_process_variance_mps2_2
        )

        # This is the bias random-walk growth over the current propagation
        # interval. It uses its own nonnegative variance-rate term rather than
        # borrowing any floor from the speed-driving acceleration noise.
        bias_random_walk_variance_mps2_2: float = dt_sec * _sanitize_process_variance(
            self._config.forward_accel_bias_process_variance_mps2_2_per_sec
        )

        self._var_forward_speed_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            previous_var_forward_speed_mps2
            - 2.0 * dt_sec * previous_cov_speed_bias_mps3
            + dt_sec
            * dt_sec
            * (
                previous_var_forward_accel_bias_mps2_2
                + accel_drive_process_variance_mps2_2
            ),
        )
        self._cov_forward_speed_bias_mps3 = (
            previous_cov_speed_bias_mps3
            - dt_sec * previous_var_forward_accel_bias_mps2_2
        )
        self._var_forward_accel_bias_mps2_2 = max(
            self._config.min_forward_accel_bias_variance_mps2_2,
            previous_var_forward_accel_bias_mps2_2 + bias_random_walk_variance_mps2_2,
        )

    def _apply_speed_measurement_update(
        self,
        *,
        measurement_value_mps: float,
        measurement_variance_mps2: float,
    ) -> None:
        innovation_mps: float = measurement_value_mps - self._forward_speed_mps
        innovation_variance_mps2: float = (
            self._var_forward_speed_mps2 + measurement_variance_mps2
        )
        kalman_gain_speed: float = (
            self._var_forward_speed_mps2 / innovation_variance_mps2
        )
        kalman_gain_bias: float = (
            self._cov_forward_speed_bias_mps3 / innovation_variance_mps2
        )

        previous_var_forward_speed_mps2: float = self._var_forward_speed_mps2
        previous_cov_speed_bias_mps3: float = self._cov_forward_speed_bias_mps3
        previous_var_forward_accel_bias_mps2_2: float = (
            self._var_forward_accel_bias_mps2_2
        )

        self._forward_speed_mps += kalman_gain_speed * innovation_mps
        self._forward_accel_bias_mps2 += kalman_gain_bias * innovation_mps
        self._var_forward_speed_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            (1.0 - kalman_gain_speed) * previous_var_forward_speed_mps2,
        )
        self._cov_forward_speed_bias_mps3 = (
            1.0 - kalman_gain_speed
        ) * previous_cov_speed_bias_mps3
        self._var_forward_accel_bias_mps2_2 = max(
            self._config.min_forward_accel_bias_variance_mps2_2,
            previous_var_forward_accel_bias_mps2_2
            - kalman_gain_bias * previous_cov_speed_bias_mps3,
        )

    def _apply_stationary_bias_update(self, *, measurement_timestamp_ns: int) -> None:
        """
        Apply a compact stationary-bias measurement model during a ZUPT.

        When the platform is declared stationary, a persistent projected
        forward acceleration is reinterpreted as bias evidence rather than real
        longitudinal motion. This is an intentionally compact heuristic
        measurement model rather than a richer inertial derivation. The latest
        projected acceleration is reused only inside a short recency window so
        the heuristic stays tied to the same physical stop event.
        """

        if self._last_forward_acceleration_timestamp_ns is None:
            return

        if self._last_forward_acceleration_mps2 is None:
            return

        sample_age_sec: float = (
            float(
                measurement_timestamp_ns - self._last_forward_acceleration_timestamp_ns
            )
            * 1.0e-9
        )
        if sample_age_sec < 0.0:
            return

        if sample_age_sec > self._config.stationary_forward_accel_bias_max_age_sec:
            return

        measurement_variance_mps2_2: float = max(
            self._config.min_forward_accel_bias_variance_mps2_2,
            self._config.stationary_forward_accel_bias_measurement_variance_mps2_2,
        )
        innovation_mps2: float = (
            self._last_forward_acceleration_mps2 - self._forward_accel_bias_mps2
        )
        innovation_variance_mps2_2: float = (
            self._var_forward_accel_bias_mps2_2 + measurement_variance_mps2_2
        )
        # The stationary bias measurement is still coupled to the speed state
        # through the speed-bias covariance term, so both states can move here
        # even though the measurement is expressed in acceleration units.
        kalman_gain_speed_from_bias_measurement_sec: float = (
            self._cov_forward_speed_bias_mps3 / innovation_variance_mps2_2
        )
        kalman_gain_bias: float = (
            self._var_forward_accel_bias_mps2_2 / innovation_variance_mps2_2
        )

        previous_var_forward_speed_mps2: float = self._var_forward_speed_mps2
        previous_cov_speed_bias_mps3: float = self._cov_forward_speed_bias_mps3
        previous_var_forward_accel_bias_mps2_2: float = (
            self._var_forward_accel_bias_mps2_2
        )

        self._forward_speed_mps += (
            kalman_gain_speed_from_bias_measurement_sec * innovation_mps2
        )
        self._forward_accel_bias_mps2 += kalman_gain_bias * innovation_mps2
        self._var_forward_speed_mps2 = max(
            self._config.min_forward_speed_variance_mps2,
            previous_var_forward_speed_mps2
            - kalman_gain_speed_from_bias_measurement_sec
            * previous_cov_speed_bias_mps3,
        )
        self._cov_forward_speed_bias_mps3 = (
            1.0 - kalman_gain_bias
        ) * previous_cov_speed_bias_mps3
        self._var_forward_accel_bias_mps2_2 = max(
            self._config.min_forward_accel_bias_variance_mps2_2,
            (1.0 - kalman_gain_bias) * previous_var_forward_accel_bias_mps2_2,
        )

    def _append_evidence_sample(self, sample: _AxisEvidenceSample) -> None:
        self._evidence_window.append(sample)
        self._window_scatter_xx += sample.flat_accel_x_mps2 * sample.flat_accel_x_mps2
        self._window_scatter_xy += sample.flat_accel_x_mps2 * sample.flat_accel_y_mps2
        self._window_scatter_yy += sample.flat_accel_y_mps2 * sample.flat_accel_y_mps2

        while len(self._evidence_window) > self._config.yaw_learning_window_size:
            oldest_sample: _AxisEvidenceSample = self._evidence_window.popleft()
            self._window_scatter_xx -= (
                oldest_sample.flat_accel_x_mps2 * oldest_sample.flat_accel_x_mps2
            )
            self._window_scatter_xy -= (
                oldest_sample.flat_accel_x_mps2 * oldest_sample.flat_accel_y_mps2
            )
            self._window_scatter_yy -= (
                oldest_sample.flat_accel_y_mps2 * oldest_sample.flat_accel_y_mps2
            )

    def _compute_candidate_axis(self) -> Optional[_CandidateAxisState]:
        sample_count: int = len(self._evidence_window)
        if sample_count == 0:
            return None

        total_energy: float = self._window_scatter_xx + self._window_scatter_yy
        if total_energy <= 0.0:
            return None

        candidate_axis_yaw_rad: float = 0.5 * math.atan2(
            2.0 * self._window_scatter_xy,
            self._window_scatter_xx - self._window_scatter_yy,
        )
        reference_yaw_rad: float = self._candidate_reference_yaw_rad()
        candidate_forward_yaw_rad: float = _canonicalize_axis_yaw(
            candidate_axis_yaw_rad,
            reference_yaw_rad=reference_yaw_rad,
        )

        discriminant: float = math.sqrt(
            max(
                0.0,
                (self._window_scatter_xx - self._window_scatter_yy)
                * (self._window_scatter_xx - self._window_scatter_yy)
                + 4.0 * self._window_scatter_xy * self._window_scatter_xy,
            )
        )
        orthogonal_energy: float = 0.5 * (total_energy - discriminant)
        confidence: float = max(0.0, min(1.0, discriminant / total_energy))
        residual_ratio: float = max(
            0.0,
            min(1.0, orthogonal_energy / total_energy),
        )
        score: float = confidence * (1.0 - residual_ratio)

        return _CandidateAxisState(
            forward_yaw_rad=candidate_forward_yaw_rad,
            confidence=confidence,
            residual_ratio=residual_ratio,
            score=score,
            sample_count=sample_count,
        )

    def _commit_candidate(
        self,
        *,
        timestamp_ns: int,
        candidate_state: _CandidateAxisState,
    ) -> None:
        self._committed_forward_yaw_rad = candidate_state.forward_yaw_rad
        self._var_committed_forward_yaw_rad2 = max(
            self._config.min_forward_yaw_variance_rad2,
            candidate_state.residual_ratio
            / max(float(candidate_state.sample_count), 1.0),
        )
        self._has_committed_forward_yaw = True
        self._checkpoint_commit_count += 1
        self._persist_forward_yaw_commit(
            timestamp_ns=timestamp_ns,
            candidate_state=candidate_state,
        )
        self._clear_candidate_evidence()
        self._turn_detected = False

    def _persist_forward_yaw_commit(
        self,
        *,
        timestamp_ns: int,
        candidate_state: _CandidateAxisState,
    ) -> None:
        persistence_path: Optional[str] = self._config.persistence_path
        if not persistence_path:
            return

        try:
            write_persisted_forward_yaw(
                path=persistence_path,
                host=self._config.persistence_host,
                created_unix_ns=timestamp_ns,
                forward_yaw_rad=self._committed_forward_yaw_rad,
                uncertainty_forward_yaw_rad=math.sqrt(
                    self._var_committed_forward_yaw_rad2
                ),
                metadata=PersistedForwardYawMetadata(
                    fit_sample_count=candidate_state.sample_count,
                    checkpoint_count=self._checkpoint_commit_count,
                    confidence=candidate_state.confidence,
                    residual=candidate_state.residual_ratio,
                    score=candidate_state.score,
                    last_update_reason="candidate_axis_checkpoint_commit",
                ),
            )
        except OSError:
            return

    def _discard_uncommitted_candidate(self) -> None:
        discarded_sample_count: int = len(self._evidence_window)
        if discarded_sample_count <= 0:
            self._clear_candidate_evidence()
            return

        self._discarded_uncommitted_sample_count += discarded_sample_count
        self._checkpoint_discard_count += 1
        self._clear_candidate_evidence()

    def _clear_candidate_evidence(self) -> None:
        self._evidence_window.clear()
        self._window_scatter_xx = 0.0
        self._window_scatter_xy = 0.0
        self._window_scatter_yy = 0.0
        self._candidate_forward_yaw_rad = None
        self._candidate_confidence = None
        self._candidate_residual_ratio = None
        self._candidate_score = None

    def _update_candidate_fields(
        self,
        candidate_state: Optional[_CandidateAxisState],
    ) -> None:
        if candidate_state is None:
            self._candidate_forward_yaw_rad = None
            self._candidate_confidence = None
            self._candidate_residual_ratio = None
            self._candidate_score = None
            return

        self._candidate_forward_yaw_rad = candidate_state.forward_yaw_rad
        self._candidate_confidence = candidate_state.confidence
        self._candidate_residual_ratio = candidate_state.residual_ratio
        self._candidate_score = candidate_state.score

    def _active_forward_yaw_rad(self) -> float:
        if self._has_committed_forward_yaw:
            return self._committed_forward_yaw_rad

        if self._candidate_forward_yaw_rad is not None:
            return self._candidate_forward_yaw_rad

        return 0.0

    def _candidate_reference_yaw_rad(self) -> float:
        if self._has_committed_forward_yaw:
            return self._committed_forward_yaw_rad

        if self._candidate_forward_yaw_rad is not None:
            return self._candidate_forward_yaw_rad

        return 0.0


def _forward_axis_flat(forward_yaw_rad: float) -> tuple[float, float, float]:
    return (
        math.cos(forward_yaw_rad),
        math.sin(forward_yaw_rad),
        0.0,
    )


def make_flat_surface_twist_covariance(
    *,
    forward_speed_mps: float,
    forward_yaw_rad: float,
    forward_speed_variance_mps2: float,
    forward_yaw_variance_rad2: float,
    yaw_rate_variance_rads2: float,
    min_forward_speed_variance_mps2: float,
    min_forward_yaw_variance_rad2: float,
    min_yaw_rate_variance_rads2: float,
) -> list[float]:
    """
    Build the yaw-only flat-surface 6x6 twist covariance.

    The estimator is fundamentally a gravity-leveled yaw-only model. The
    published twist may be expressed in `base_link`, but linear covariance
    remains constrained to the flat-surface x-y subspace with exact zeros in
    the z row and z column.
    """

    covariance: list[float] = [0.0] * 36
    axis_x: float = math.cos(forward_yaw_rad)
    axis_y: float = math.sin(forward_yaw_rad)
    lateral_x: float = -axis_y
    lateral_y: float = axis_x
    scalar_variance_mps2: float = _sanitize_positive_variance(
        forward_speed_variance_mps2,
        minimum_variance=min_forward_speed_variance_mps2,
    )
    yaw_variance_rad2: float = _sanitize_positive_variance(
        forward_yaw_variance_rad2,
        minimum_variance=min_forward_yaw_variance_rad2,
    )
    yaw_linear_variance_mps2: float = (
        yaw_variance_rad2 * float(forward_speed_mps) * float(forward_speed_mps)
    )
    yaw_rate_variance_value_rads2: float = _sanitize_positive_variance(
        yaw_rate_variance_rads2,
        minimum_variance=min_yaw_rate_variance_rads2,
    )

    covariance[0] = (
        scalar_variance_mps2 * axis_x * axis_x
        + yaw_linear_variance_mps2 * lateral_x * lateral_x
    )
    covariance[1] = (
        scalar_variance_mps2 * axis_x * axis_y
        + yaw_linear_variance_mps2 * lateral_x * lateral_y
    )
    covariance[6] = covariance[1]
    covariance[7] = (
        scalar_variance_mps2 * axis_y * axis_y
        + yaw_linear_variance_mps2 * lateral_y * lateral_y
    )
    covariance[35] = yaw_rate_variance_value_rads2
    return covariance


def _project_body_vector_onto_world_flat_surface(
    *,
    vector_body: tuple[float, float, float],
    rotation_world_to_base: tuple[tuple[float, float, float], ...],
    rotation_base_to_world: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    """
    Project a body-frame vector onto the world-horizontal flat surface.

    Orientation is used only to define the gravity-leveled surface. The
    returned vector is still expressed in `base_link`, which keeps the
    estimator yaw-only while avoiding a free 3D axis fit.
    """

    vector_world: tuple[float, float, float] = rotate_vector(
        rotation_base_to_world,
        vector_body,
    )
    vector_world_flat: tuple[float, float, float] = (
        vector_world[0],
        vector_world[1],
        0.0,
    )
    return rotate_vector(rotation_world_to_base, vector_world_flat)


def _canonicalize_axis_yaw(
    axis_yaw_rad: float,
    *,
    reference_yaw_rad: float,
) -> float:
    base_yaw_rad: float = _normalize_angle(axis_yaw_rad)
    alternate_yaw_rad: float = _normalize_angle(base_yaw_rad + math.pi)
    if abs(_normalize_angle(alternate_yaw_rad - reference_yaw_rad)) < abs(
        _normalize_angle(base_yaw_rad - reference_yaw_rad)
    ):
        return alternate_yaw_rad

    return base_yaw_rad


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _sanitize_positive_variance(
    variance_value: float,
    *,
    minimum_variance: float,
) -> float:
    sanitized_value: float = float(variance_value)
    if not math.isfinite(sanitized_value) or sanitized_value <= 0.0:
        return float(minimum_variance)

    return sanitized_value


def _sanitize_process_variance(variance_value: float) -> float:
    """
    Clamp one process-noise variance term to a finite nonnegative value.

    Process-noise terms describe injected uncertainty, not state-variance
    floors, so invalid values collapse to `0.0` instead of a state minimum.
    """

    sanitized_value: float = float(variance_value)
    if not math.isfinite(sanitized_value) or sanitized_value <= 0.0:
        return 0.0

    return sanitized_value


def _dot(
    lhs_vector: tuple[float, float, float],
    rhs_vector: tuple[float, float, float],
) -> float:
    return (
        lhs_vector[0] * rhs_vector[0]
        + lhs_vector[1] * rhs_vector[1]
        + lhs_vector[2] * rhs_vector[2]
    )
