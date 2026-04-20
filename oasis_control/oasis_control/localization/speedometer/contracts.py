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

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ForwardTwistConfig:
    """
    Tunable parameters for the AHRS forward-twist estimator.

    Fields:
        expected_imu_frame_id: frame_id expected on incoming `ahrs/imu`
            samples
        output_frame_id: frame_id used for published `ahrs/forward_twist`
            messages
        learning_accel_threshold_mps2: minimum horizontal body-frame
            acceleration magnitude accepted as learning evidence
        learning_min_samples: minimum accepted samples required before a
            checkpoint may commit
        learning_min_confidence: minimum sign-aligned directional consistency
            in [0, 1] required for candidate checkpoint commits
        checkpoint_min_samples: minimum uncommitted sample count required for a
            new checkpoint commit
        checkpoint_max_candidate_delta_rad: maximum allowed yaw change between
            the committed and candidate axes for an incremental checkpoint
        initial_forward_speed_sigma_mps: initial 1-sigma uncertainty on the
            scalar forward speed state
        min_forward_speed_variance_mps2: minimum published scalar speed
            variance in m^2/s^2
        forward_accel_process_sigma_mps2: owned 1-sigma acceleration process
            noise projected along the learned forward axis
        max_imu_dt_sec: maximum positive IMU propagation interval in seconds
            accepted by the deterministic speed propagation step
        turn_rate_threshold_rads: yaw-rate magnitude above which the fused turn
            detector immediately gates learning
        turn_accel_threshold_mps2: minimum horizontal acceleration magnitude
            required before acceleration-direction evidence contributes to turn
            gating
        turn_direction_alignment_threshold: minimum sign-aligned horizontal
            acceleration directional agreement in [0, 1] before the fused
            detector treats motion as inconsistent with straight-axis learning
        zupt_freshness_window_sec: maximum allowed age between the latest IMU
            sample and a candidate ZUPT or `zupt_flag` sample before the
            zero-speed correction is rejected as stale
        zupt_motion_reject_accel_threshold_mps2: projected forward-axis
            acceleration magnitude above which a fresh ZUPT is treated as
            contradictory to current motion and rejected
        zupt_motion_reject_speed_threshold_mps: signed speed magnitude above
            which a contradictory fresh ZUPT is rejected more aggressively
        persistence_write_on_checkpoint: true when committed checkpoints should
            be persisted to disk
    """

    expected_imu_frame_id: str = "base_link"
    output_frame_id: str = "base_link"
    learning_accel_threshold_mps2: float = 0.10
    learning_min_samples: int = 4
    learning_min_confidence: float = 0.70
    checkpoint_min_samples: int = 3
    checkpoint_max_candidate_delta_rad: float = 0.80
    initial_forward_speed_sigma_mps: float = 3.0
    min_forward_speed_variance_mps2: float = 1.0e-4
    forward_accel_process_sigma_mps2: float = 0.75
    max_imu_dt_sec: float = 0.25
    turn_rate_threshold_rads: float = 0.35
    turn_accel_threshold_mps2: float = 0.35
    turn_direction_alignment_threshold: float = 0.55
    zupt_freshness_window_sec: float = 0.20
    zupt_motion_reject_accel_threshold_mps2: float = 0.35
    zupt_motion_reject_speed_threshold_mps: float = 0.15
    persistence_write_on_checkpoint: bool = True


@dataclass(frozen=True)
class ForwardAxisState:
    """
    Public forward-axis state.

    Fields:
        forward_yaw_rad: learned yaw angle in radians in the node output frame
        forward_axis_xyz: public unit forward direction `[cos(yaw), sin(yaw),
            0]`
        learned: true when the committed state should be treated as usable
    """

    forward_yaw_rad: float
    forward_axis_xyz: tuple[float, float, float]
    learned: bool


@dataclass(frozen=True)
class LearningState:
    """
    Online learning state split into candidate and committed tracks.

    Fields:
        candidate_forward_yaw_rad: tentative online yaw estimate in radians
        committed_forward_yaw_rad: public persisted yaw estimate in radians
        candidate_sample_count: total accepted samples represented by the
            current candidate yaw
        committed_sample_count: total samples represented by committed
            checkpoints
        uncommitted_sample_count: accepted samples buffered since the last
            checkpoint
        candidate_confidence: directional consistency of the candidate evidence
            in [0, 1]
        forward_sign_locked: true once the first accepted motion sample has
            defined positive forward direction
        learning_gated_by_turn: true when turns are currently blocking
            candidate updates
        checkpoint_commit_count: number of committed checkpoints published so
            far
        checkpoint_discard_count: number of uncommitted checkpoint buffers
            discarded due to turn gating
        checkpoint_just_committed: true only on the estimate emitted by a new
            checkpoint commit
        committed_confidence: confidence of the active committed estimate in
            [0, 1]
        committed_score: long-term score used to decide whether a candidate is
            genuinely better than the current committed estimate
        candidate_score: score of the latest candidate estimate
        committed_residual: residual mismatch proxy for the committed estimate
        candidate_residual: residual mismatch proxy for the latest candidate
            estimate
        committed_uncertainty_forward_yaw_rad: approximate 1-sigma yaw
            uncertainty of the committed estimate in radians
        candidate_uncertainty_forward_yaw_rad: approximate 1-sigma yaw
            uncertainty of the candidate estimate in radians
        candidate_beats_committed: true when the latest candidate score passes
            the deterministic "better estimate" rule
        last_commit_reason: short explanation for why the last committed value
            was accepted or preserved
        committed_source: one of `learning` or `persistence`
    """

    candidate_forward_yaw_rad: float
    committed_forward_yaw_rad: float
    candidate_sample_count: int
    committed_sample_count: int
    uncommitted_sample_count: int
    candidate_confidence: float
    forward_sign_locked: bool
    learning_gated_by_turn: bool
    checkpoint_commit_count: int
    checkpoint_discard_count: int
    checkpoint_just_committed: bool
    committed_confidence: float
    committed_score: float
    candidate_score: float
    committed_residual: float
    candidate_residual: float
    committed_uncertainty_forward_yaw_rad: float
    candidate_uncertainty_forward_yaw_rad: float
    candidate_beats_committed: bool
    last_commit_reason: str
    committed_source: str


@dataclass(frozen=True)
class ForwardTwistEstimate:
    """
    Placeholder forward-twist output state.

    Fields:
        timestamp_ns: source sample timestamp in ns
        forward_speed_mps: scalar speed along the public forward axis in m/s
        forward_speed_variance_mps2: scalar speed variance in m^2/s^2
        forward_speed_sigma_mps: scalar speed 1-sigma in m/s for HUD use
        forward_axis: learned public forward-axis state
        learning_state: candidate-versus-committed learning snapshot
        turn_detected: true when the latest IMU sample was classified as a turn
        latest_zupt_flag_age_sec: age in seconds between the newest IMU sample
            and the most recent `zupt_flag`, or None when unavailable
        latest_zupt_age_sec: age in seconds between the newest IMU sample and
            the most recent `zupt`, or None when unavailable
        zupt_update_applied: true when the latest ZUPT correction changed the
            scalar speed state
        zupt_rejected_stale: true when the latest candidate ZUPT correction was
            rejected because the `zupt` or its paired flag were too old
        zupt_rejected_motion_contradiction: true when the latest candidate ZUPT
            correction was rejected because recent IMU motion contradicted
            stationarity
        zupt_measurement_variance_used_mps2: effective scalar zero-speed
            variance used for the latest candidate correction, or None when no
            fresh candidate was available
        zupt_kalman_gain: scalar Kalman gain used for the latest applied
            zero-speed correction, or 0 when none was applied
        zupt_applied_count: cumulative count of applied zero-speed corrections
        zupt_rejected_stale_count: cumulative count of stale ZUPT rejections
        zupt_rejected_motion_count: cumulative count of contradictory-motion
            ZUPT rejections
        startup_loaded_from_persistence: true when the current committed value
            was initialized from a persisted file on boot
        persistence_load_path: path of the attempted startup load file
        persistence_load_valid: true when startup load succeeded
        persistence_load_error: load failure reason, if any
        current_commit_from_persistence: true when the current committed yaw
            still originates from the loaded persisted estimate
        persistence_write_count: cumulative count of successful persistence
            writes for committed improvements
        last_persistence_reason: explanation for the last persistence write or
            startup-load fallback
        imu_sample_rejected: true when the triggering IMU update was rejected
            or deterministically dropped
        zupt_sample_rejected: true when the triggering ZUPT update was
            rejected or ignored
    """

    timestamp_ns: int
    forward_speed_mps: float
    forward_speed_variance_mps2: float
    forward_speed_sigma_mps: float
    forward_axis: ForwardAxisState
    learning_state: LearningState
    turn_detected: bool
    latest_zupt_flag_age_sec: Optional[float]
    latest_zupt_age_sec: Optional[float]
    zupt_update_applied: bool
    zupt_rejected_stale: bool
    zupt_rejected_motion_contradiction: bool
    zupt_measurement_variance_used_mps2: Optional[float]
    zupt_kalman_gain: float
    zupt_applied_count: int
    zupt_rejected_stale_count: int
    zupt_rejected_motion_count: int
    startup_loaded_from_persistence: bool
    persistence_load_path: str
    persistence_load_valid: bool
    persistence_load_error: str
    current_commit_from_persistence: bool
    persistence_write_count: int
    last_persistence_reason: str
    imu_sample_rejected: bool
    zupt_sample_rejected: bool


@dataclass(frozen=True)
class TurnDetection:
    """
    Fused turn-detector output.

    Fields:
        turn_detected: true when learning should currently be gated
        yaw_rate_rads: measured yaw rate in rad/s used by the gate
        horizontal_accel_mps2: horizontal body-frame acceleration magnitude in
            m/s^2 used by the acceleration-behavior check
        reference_alignment: sign-aligned agreement in [0, 1] between the
            recent horizontal acceleration direction and the running reference
        threshold_rads: configured yaw-rate threshold in rad/s
    """

    turn_detected: bool
    yaw_rate_rads: float
    horizontal_accel_mps2: float
    reference_alignment: float
    threshold_rads: float


@dataclass(frozen=True)
class PersistenceRecord:
    """
    Persisted committed forward-yaw payload.

    Fields:
        version: persistence schema version
        created_unix_ns: write timestamp in Unix nanoseconds
        hostname: host identifier encoded in the persistence filename/payload
        estimator: producer identifier for debugging and future migrations
        valid: true when the committed forward axis should be treated as usable
        forward_yaw_rad: committed public yaw in radians
        forward_axis_xyz: committed public forward axis `[cos(yaw), sin(yaw),
            0]`
        fit_sample_count: total accepted learning samples represented by the
            committed fit
        checkpoint_count: number of committed checkpoints represented by this
            payload
        confidence: confidence of the committed estimate in [0, 1]
        score: deterministic quality score for the committed estimate
        residual: residual mismatch proxy for the committed estimate
        uncertainty_forward_yaw_rad: approximate committed 1-sigma yaw
            uncertainty in radians
        loaded_startup_capable: true when the payload is suitable for direct
            startup use
        last_update_reason: explanation for why this persisted value replaced
            the prior committed estimate
    """

    version: int
    created_unix_ns: int
    hostname: str
    estimator: str
    valid: bool
    forward_yaw_rad: float
    forward_axis_xyz: tuple[float, float, float]
    fit_sample_count: int
    checkpoint_count: int
    confidence: float = 0.0
    score: float = 0.0
    residual: float = 1.0
    uncertainty_forward_yaw_rad: float = 3.141592653589793
    loaded_startup_capable: bool = True
    last_update_reason: str = ""


@dataclass(frozen=True)
class ZuptMeasurement:
    """
    Placeholder ZUPT measurement contract.

    Fields:
        timestamp_ns: measurement timestamp in ns
        zero_velocity_variance_mps2: scalar zero-velocity variance in m^2/s^2
        stationary_flag: optional stationarity flag paired from `zupt_flag`
        stationary_flag_timestamp_ns: timestamp of the paired `zupt_flag`, or
            None when unavailable
    """

    timestamp_ns: int
    zero_velocity_variance_mps2: float
    stationary_flag: Optional[bool]
    stationary_flag_timestamp_ns: Optional[int] = None
