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
    Placeholder configuration for the forward-twist skeleton.

    Fields:
        expected_imu_frame_id: frame_id expected on incoming `ahrs/imu`
            samples
        output_frame_id: frame_id used for published `ahrs/forward_twist`
            messages
        committed_learning_gain: placeholder scalar in [0, 1] that controls
            how quickly candidate yaw is promoted toward the committed state
        candidate_learning_gain: placeholder scalar in [0, 1] that controls
            how quickly online samples nudge the candidate forward yaw
        min_forward_speed_variance_mps2: minimum published scalar speed
            variance in m^2/s^2
        turn_rate_threshold_rads: yaw-rate magnitude above which learning is
            gated as "in turn"
    """

    expected_imu_frame_id: str = "base_link"
    output_frame_id: str = "base_link"
    committed_learning_gain: float = 0.05
    candidate_learning_gain: float = 0.10
    min_forward_speed_variance_mps2: float = 1.0e-4
    turn_rate_threshold_rads: float = 0.35


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
        candidate_sample_count: number of accepted online learning updates
        committed_sample_count: number of candidate promotions committed so far
        learning_gated_by_turn: true when turns are currently blocking
            candidate updates
    """

    candidate_forward_yaw_rad: float
    committed_forward_yaw_rad: float
    candidate_sample_count: int
    committed_sample_count: int
    learning_gated_by_turn: bool


@dataclass(frozen=True)
class ForwardTwistEstimate:
    """
    Placeholder forward-twist output state.

    Fields:
        timestamp_ns: source sample timestamp in ns
        forward_speed_mps: scalar speed along the public forward axis in m/s
        forward_speed_variance_mps2: scalar speed variance in m^2/s^2
        forward_axis: learned public forward-axis state
        learning_state: candidate-versus-committed learning snapshot
        turn_detected: true when the latest IMU sample was classified as a turn
    """

    timestamp_ns: int
    forward_speed_mps: float
    forward_speed_variance_mps2: float
    forward_axis: ForwardAxisState
    learning_state: LearningState
    turn_detected: bool


@dataclass(frozen=True)
class TurnDetection:
    """
    Placeholder turn-detector output.

    Fields:
        turn_detected: true when learning should currently be gated
        yaw_rate_rads: measured yaw rate in rad/s used by the gate
        threshold_rads: configured turn threshold in rad/s
    """

    turn_detected: bool
    yaw_rate_rads: float
    threshold_rads: float


@dataclass(frozen=True)
class PersistenceRecord:
    """
    Minimal persisted forward-yaw payload.

    Fields:
        hostname: host identifier encoded in the persistence filename/payload
        forward_yaw_rad: committed public yaw in radians
    """

    hostname: str
    forward_yaw_rad: float


@dataclass(frozen=True)
class ZuptMeasurement:
    """
    Placeholder ZUPT measurement contract.

    Fields:
        timestamp_ns: measurement timestamp in ns
        zero_velocity_variance_mps2: scalar zero-velocity variance in m^2/s^2
        stationary_flag: optional stationarity flag paired from `zupt_flag`
    """

    timestamp_ns: int
    zero_velocity_variance_mps2: float
    stationary_flag: Optional[bool]
