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
    Configuration for the yaw-only flat-surface forward-twist estimator.

    Fields:
        expected_imu_frame_id: frame_id expected on incoming `ahrs/imu`
            samples
        output_frame_id: frame_id used for published `ahrs/forward_twist`
            messages
        min_forward_speed_variance_mps2: minimum published scalar speed
            variance in m^2/s^2
        min_forward_yaw_variance_rad2: minimum learned forward-yaw variance in
            rad^2
        min_yaw_rate_variance_rads2: minimum owned yaw-rate variance in
            rad^2/s^2
        forward_accel_process_variance_mps2_2: scalar forward-acceleration
            process-noise variance in (m/s^2)^2
        default_yaw_rate_variance_rads2: owned default yaw-rate variance in
            rad^2/s^2
        yaw_learning_accel_threshold_mps2: minimum flat-surface acceleration
            magnitude required to use one sample for forward-yaw learning
        yaw_learning_window_size: maximum accepted flat-surface learning
            samples retained in the current candidate evidence window
        yaw_learning_max_yaw_rate_rads: maximum yaw-rate magnitude allowed for
            yaw-learning evidence accumulation
        yaw_learning_min_samples: minimum accepted learning samples required
            before committing a learned forward yaw
        yaw_learning_min_confidence: minimum dominant-axis confidence required
            before committing a learned forward yaw
        yaw_learning_max_residual_ratio: maximum orthogonal-energy ratio
            allowed for a committed candidate checkpoint
        persistence_path: optional YAML path for persisted `forward_yaw`
        persistence_host: hostname metadata stored in persisted YAML
    """

    expected_imu_frame_id: str = "base_link"
    output_frame_id: str = "base_link"
    min_forward_speed_variance_mps2: float = 1.0e-4
    min_forward_yaw_variance_rad2: float = 1.0e-4
    min_yaw_rate_variance_rads2: float = 1.0e-4
    forward_accel_process_variance_mps2_2: float = 4.0
    default_yaw_rate_variance_rads2: float = 0.05
    yaw_learning_accel_threshold_mps2: float = 0.2
    yaw_learning_window_size: int = 20
    yaw_learning_max_yaw_rate_rads: float = 0.2
    yaw_learning_min_samples: int = 10
    yaw_learning_min_confidence: float = 0.75
    yaw_learning_max_residual_ratio: float = 0.2
    persistence_path: Optional[str] = None
    persistence_host: str = ""


@dataclass(frozen=True)
class ZuptMeasurement:
    """
    Scalar zero-speed correction derived from a 6D stationary-twist ZUPT input.

    Fields:
        timestamp_ns: measurement timestamp in ns
        zero_velocity_variance_mps2: scalar variance in m^2/s^2 for the zero
            forward-speed correction extracted from the leading `3 x 3` linear
            covariance block of the stationary-twist measurement
        stationary_flag: optional boolean paired from `zupt_flag`
    """

    timestamp_ns: int
    zero_velocity_variance_mps2: float
    stationary_flag: Optional[bool]


@dataclass(frozen=True)
class ForwardTwistEstimate:
    """
    Runtime forward-twist estimate for the yaw-only flat-surface model.

    Fields:
        timestamp_ns: source timestamp in ns for the latest runtime update
        forward_yaw_rad: active forward yaw in radians on the `body_gravity`
            flat-surface frame used for runtime output
        committed_forward_yaw_rad: trusted committed forward yaw in rad
        committed_forward_yaw_variance_rad2: committed forward-yaw variance in
            rad^2
        candidate_forward_yaw_rad: candidate forward yaw in rad or None when no
            candidate evidence window is active
        candidate_confidence: dominant-axis confidence for the current
            candidate or None when unavailable
        candidate_residual_ratio: orthogonal-energy ratio for the current
            candidate or None when unavailable
        forward_speed_mps: scalar forward speed in m/s
        forward_speed_variance_mps2: scalar forward-speed variance in m^2/s^2
        yaw_rate_rads: scalar yaw angular velocity in rad/s
        yaw_rate_variance_rads2: scalar yaw-rate variance in rad^2/s^2
        accepted_learning_sample_count: cumulative accepted learning samples
        discarded_uncommitted_sample_count: cumulative discarded candidate
            learning samples
        checkpoint_commit_count: number of committed candidate checkpoints
        checkpoint_discard_count: number of discarded candidate checkpoints
        turn_detected: true when current learning evidence is gated by the
            compact turn detector
        loaded_from_persistence: true when authoritative `forward_yaw` was
            loaded from persistence for this run
        learning_enabled: true when forward-yaw learning is still enabled for
            this run
    """

    timestamp_ns: int
    forward_yaw_rad: float
    committed_forward_yaw_rad: float
    committed_forward_yaw_variance_rad2: float
    candidate_forward_yaw_rad: Optional[float]
    candidate_confidence: Optional[float]
    candidate_residual_ratio: Optional[float]
    forward_speed_mps: float
    forward_speed_variance_mps2: float
    yaw_rate_rads: float
    yaw_rate_variance_rads2: float
    accepted_learning_sample_count: int
    discarded_uncommitted_sample_count: int
    checkpoint_commit_count: int
    checkpoint_discard_count: int
    turn_detected: bool
    loaded_from_persistence: bool
    learning_enabled: bool
