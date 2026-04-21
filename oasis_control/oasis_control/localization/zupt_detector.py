################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Stationary-twist detector based on IMU norm thresholds and dwell timers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


Vector3 = tuple[float, float, float]


@dataclass
class ZuptDetectorConfig:
    """Configuration values for the IMU-only ZUPT detector."""

    # Conservative gyro norm threshold in rad/s required to enter
    # stationary mode after quiet dwell
    gyro_enter_threshold_rads: float = 0.06

    # Aggressive gyro norm threshold in rad/s required to exit
    # stationary mode on raw motion onset
    gyro_exit_threshold_rads: float = 0.09

    # Conservative linear-acceleration norm threshold in m/s^2
    # required to enter stationary mode after quiet dwell
    accel_enter_threshold_mps2: float = 0.18

    # Aggressive linear-acceleration norm threshold in m/s^2
    # required to exit stationary mode on raw motion onset
    accel_exit_threshold_mps2: float = 0.28

    # Minimum time in seconds that quiet IMU conditions must hold
    # before asserting stationary
    min_stationary_sec: float = 0.18

    # Minimum time in seconds that raw moving IMU conditions must hold
    # before clearing stationary. This is intentionally short so brief
    # real motion clears stationary faster than stationary asserts
    min_moving_sec: float = 0.01

    # Stationary linear-velocity sigma in m/s used for the isotropic
    # `3 x 3` linear covariance block
    stationary_linear_velocity_sigma_mps: float = 0.06

    # Stationary angular-velocity sigma in rad/s used for the isotropic
    # `3 x 3` angular covariance block
    stationary_angular_velocity_sigma_rads: float = 0.06

    # Moving linear-velocity variance in (m/s)^2 so downstream
    # stationary-twist updates become negligible while motion remains
    # explicit
    moving_linear_variance_mps2: float = 1.0e6

    # Moving angular-velocity variance in (rad/s)^2 so downstream
    # stationary-twist updates become negligible while motion remains
    # explicit
    moving_angular_variance_rads2: float = 1.0e6


@dataclass
class ZuptDetectorState:
    """Mutable state for the ZUPT detector."""

    # True when the detector currently considers the platform stationary
    stationary: bool = False

    # Timestamp in seconds when the current enter candidate began
    enter_candidate_start_sec: Optional[float] = None

    # Timestamp in seconds when the current exit candidate began
    exit_candidate_start_sec: Optional[float] = None

    # Last processed timestamp in seconds
    last_timestamp_sec: Optional[float] = None

    # Last gyro norm in rad/s
    last_gyro_norm_rads: float = 0.0

    # Last linear-acceleration norm in m/s^2 from the upstream IMU field
    last_accel_norm_mps2: float = 0.0

    # Current published linear stationary-twist variance in (m/s)^2
    current_linear_zupt_variance_mps2: float = 0.0

    # Current published angular stationary-twist variance in (rad/s)^2
    current_angular_zupt_variance_rads2: float = 0.0

    # Short reason describing the last state transition decision
    last_reason: str = "init"


@dataclass(frozen=True)
class ZuptDecision:
    """Detector output for a single IMU sample."""

    # True when the platform is considered stationary
    stationary: bool

    # Angular-velocity norm in rad/s from the current IMU sample
    gyro_norm_rads: float

    # Linear-acceleration norm in m/s^2 from the current upstream IMU
    # sample
    accel_norm_mps2: float

    # Published linear stationary-twist variance in (m/s)^2 carried in
    # the leading `3 x 3` covariance block
    linear_zupt_variance_mps2: float

    # Published angular stationary-twist variance in (rad/s)^2 carried
    # in the trailing `3 x 3` covariance block
    angular_zupt_variance_rads2: float

    # Time in seconds that the current enter candidate has been active
    enter_dwell_sec: float

    # Time in seconds that the current exit candidate has been active
    exit_dwell_sec: float

    # Short reason describing the current detector decision
    reason: str


class ZuptDetector:
    """Detect stationary intervals and publish a paired stationary-twist."""

    def __init__(self, config: ZuptDetectorConfig) -> None:
        self._config: ZuptDetectorConfig = config
        self._state: ZuptDetectorState = ZuptDetectorState(
            current_linear_zupt_variance_mps2=config.moving_linear_variance_mps2,
            current_angular_zupt_variance_rads2=config.moving_angular_variance_rads2,
        )

    @property
    def state(self) -> ZuptDetectorState:
        """Return the mutable detector state."""

        return self._state

    def update(
        self,
        timestamp_sec: float,
        angular_velocity_rads: Vector3,
        linear_accel_mps2: Vector3,
    ) -> Optional[ZuptDecision]:
        """Update the detector using one IMU sample."""

        timestamp: float = float(timestamp_sec)
        gyro_vector_rads: Vector3 = _coerce_vector3(angular_velocity_rads)
        accel_vector_mps2: Vector3 = _coerce_vector3(linear_accel_mps2)

        previous_timestamp_sec: Optional[float] = self._state.last_timestamp_sec
        if not math.isfinite(timestamp):
            self._state.last_reason = "invalid_timestamp"
            return None

        if previous_timestamp_sec is not None and timestamp < previous_timestamp_sec:
            self._state.enter_candidate_start_sec = None
            self._state.exit_candidate_start_sec = None
            self._state.last_reason = "non_monotonic_timestamp"
            return None

        self._state.last_timestamp_sec = timestamp
        if not _vector3_is_finite(gyro_vector_rads) or not _vector3_is_finite(
            accel_vector_mps2
        ):
            self._state.last_reason = "invalid_imu_sample"
            return self._build_decision(self._state.last_reason)

        gyro_norm_rads: float = _norm3(gyro_vector_rads)
        accel_norm_mps2: float = _norm3(accel_vector_mps2)
        self._state.last_gyro_norm_rads = gyro_norm_rads
        self._state.last_accel_norm_mps2 = accel_norm_mps2

        enter_candidate: bool = (
            gyro_norm_rads <= self._config.gyro_enter_threshold_rads
            and accel_norm_mps2 <= self._config.accel_enter_threshold_mps2
        )

        # Exit intentionally uses raw norm threshold breaches with a
        # very short dwell so motion onset clears stationary quickly.
        exit_candidate: bool = (
            gyro_norm_rads >= self._config.gyro_exit_threshold_rads
            or accel_norm_mps2 >= self._config.accel_exit_threshold_mps2
        )

        if self._state.stationary:
            self._update_exit_candidate(
                timestamp_sec=timestamp,
                previous_timestamp_sec=previous_timestamp_sec,
                exit_candidate=exit_candidate,
            )
        else:
            self._update_enter_candidate(
                timestamp_sec=timestamp,
                enter_candidate=enter_candidate,
            )

        self._state.current_linear_zupt_variance_mps2 = (
            self._stationary_linear_variance_mps2()
            if self._state.stationary
            else float(self._config.moving_linear_variance_mps2)
        )
        self._state.current_angular_zupt_variance_rads2 = (
            self._stationary_angular_variance_rads2()
            if self._state.stationary
            else float(self._config.moving_angular_variance_rads2)
        )
        return self._build_decision(self._state.last_reason)

    def _update_enter_candidate(
        self, timestamp_sec: float, enter_candidate: bool
    ) -> None:
        if not enter_candidate:
            self._state.enter_candidate_start_sec = None
            self._state.last_reason = "moving"
            return

        if self._state.enter_candidate_start_sec is None:
            self._state.enter_candidate_start_sec = timestamp_sec
            self._state.last_reason = "enter_candidate_started"
            return

        enter_dwell_sec: float = timestamp_sec - self._state.enter_candidate_start_sec
        if _duration_reached(enter_dwell_sec, self._config.min_stationary_sec):
            self._state.stationary = True
            self._state.enter_candidate_start_sec = None
            self._state.exit_candidate_start_sec = None
            self._state.last_reason = "stationary_asserted"
            return

        self._state.last_reason = "enter_candidate_pending"

    def _update_exit_candidate(
        self,
        timestamp_sec: float,
        previous_timestamp_sec: Optional[float],
        exit_candidate: bool,
    ) -> None:
        if not exit_candidate:
            self._state.exit_candidate_start_sec = None
            self._state.last_reason = "stationary_held"
            return

        if self._state.exit_candidate_start_sec is None:
            self._state.exit_candidate_start_sec = (
                previous_timestamp_sec
                if previous_timestamp_sec is not None
                else timestamp_sec
            )

        exit_dwell_sec: float = timestamp_sec - self._state.exit_candidate_start_sec
        if _duration_reached(exit_dwell_sec, self._config.min_moving_sec):
            self._state.stationary = False
            self._state.enter_candidate_start_sec = None
            self._state.exit_candidate_start_sec = None
            self._state.last_reason = "stationary_cleared"
            return

        if previous_timestamp_sec == self._state.exit_candidate_start_sec:
            self._state.last_reason = "exit_candidate_started"
            return

        self._state.last_reason = "exit_candidate_pending"

    def _stationary_linear_variance_mps2(self) -> float:
        return self._config.stationary_linear_velocity_sigma_mps**2

    def _stationary_angular_variance_rads2(self) -> float:
        return self._config.stationary_angular_velocity_sigma_rads**2

    def _build_decision(self, reason: str) -> ZuptDecision:
        timestamp_sec: Optional[float] = self._state.last_timestamp_sec
        enter_dwell_sec: float = _elapsed_sec(
            timestamp_sec, self._state.enter_candidate_start_sec
        )
        exit_dwell_sec: float = _elapsed_sec(
            timestamp_sec, self._state.exit_candidate_start_sec
        )
        return ZuptDecision(
            stationary=self._state.stationary,
            gyro_norm_rads=self._state.last_gyro_norm_rads,
            accel_norm_mps2=self._state.last_accel_norm_mps2,
            linear_zupt_variance_mps2=self._state.current_linear_zupt_variance_mps2,
            angular_zupt_variance_rads2=(
                self._state.current_angular_zupt_variance_rads2
            ),
            enter_dwell_sec=enter_dwell_sec,
            exit_dwell_sec=exit_dwell_sec,
            reason=reason,
        )


def _coerce_vector3(values: Vector3) -> Vector3:
    return (float(values[0]), float(values[1]), float(values[2]))


def _vector3_is_finite(values: Vector3) -> bool:
    return (
        math.isfinite(values[0])
        and math.isfinite(values[1])
        and math.isfinite(values[2])
    )


def _norm3(values: Vector3) -> float:
    return math.sqrt(
        values[0] * values[0] + values[1] * values[1] + values[2] * values[2]
    )


def _elapsed_sec(
    timestamp_sec: Optional[float], candidate_start_sec: Optional[float]
) -> float:
    if timestamp_sec is None or candidate_start_sec is None:
        return 0.0
    if timestamp_sec < candidate_start_sec:
        return 0.0
    return timestamp_sec - candidate_start_sec


def _duration_reached(elapsed_sec: float, threshold_sec: float) -> bool:
    if elapsed_sec >= threshold_sec:
        return True

    return math.isclose(elapsed_sec, threshold_sec, rel_tol=0.0, abs_tol=1.0e-12)
