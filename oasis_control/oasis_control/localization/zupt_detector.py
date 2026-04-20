################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Zero-velocity update (ZUPT) detector based on IMU data."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


Vector3 = tuple[float, float, float]


@dataclass
class ZuptDetectorConfig:
    """Configuration values for the IMU-only ZUPT detector."""

    # Gyro norm threshold in rad/s used to enter stationary mode
    gyro_enter_threshold_rads: float = 0.06

    # Gyro norm threshold in rad/s used to exit stationary mode
    gyro_exit_threshold_rads: float = 0.09

    # Linear-acceleration norm threshold in m/s^2 used to enter
    # stationary mode
    accel_enter_threshold_mps2: float = 0.18

    # Linear-acceleration norm threshold in m/s^2 used to exit
    # stationary mode
    accel_exit_threshold_mps2: float = 0.28

    # Minimum time in seconds that quiet IMU conditions must hold before
    # asserting stationary
    min_stationary_sec: float = 0.18

    # Minimum time in seconds that motion conditions must hold before
    # clearing stationary
    min_moving_sec: float = 0.1

    # Exponential smoothing time constant in seconds used for
    # stationary-enter evidence. Non-positive values disable
    # smoothing and make entry depend on the raw norms.
    enter_smoothing_time_constant_sec: float = 0.12

    # Base zero-velocity measurement sigma in m/s while stationary
    zupt_velocity_sigma_mps: float = 0.06

    # Non-stationary covariance published in (m/s)^2 so downstream ZUPT
    # updates become negligible while state remains explicit
    moving_zupt_variance_mps2: float = 1.0e6

    # Maximum multiplier applied to stationary variance when the IMU
    # norms approach the exit thresholds
    stationary_variance_inflation: float = 4.0


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

    # Last gravity-removed acceleration norm in m/s^2
    last_accel_norm_mps2: float = 0.0

    # Last filtered gyro norm in rad/s used for stationary entry
    last_filtered_gyro_norm_rads: float = 0.0

    # Last filtered gravity-removed acceleration norm in m/s^2 used
    # for stationary entry
    last_filtered_accel_norm_mps2: float = 0.0

    # Current published ZUPT variance in (m/s)^2
    current_zupt_variance_mps2: float = 0.0

    # Short reason describing the last state transition decision
    last_reason: str = "init"

    # Evidence source used for the current stationary-enter decision
    enter_evidence_source: str = "raw"


@dataclass(frozen=True)
class ZuptDecision:
    """Detector output for a single IMU sample."""

    # True when the platform is considered stationary
    stationary: bool

    # Angular-velocity norm in rad/s from the current IMU sample
    gyro_norm_rads: float

    # Gravity-removed linear-acceleration norm in m/s^2 from the
    # current IMU sample
    accel_norm_mps2: float

    # Filtered angular-velocity norm in rad/s used for stationary
    # entry decisions
    filtered_gyro_norm_rads: float

    # Filtered gravity-removed acceleration norm in m/s^2 used for
    # stationary entry decisions
    filtered_accel_norm_mps2: float

    # Published ZUPT variance in (m/s)^2 carried in covariance[0]
    zupt_variance_mps2: float

    # Time in seconds that the current enter candidate has been active
    enter_dwell_sec: float

    # Time in seconds that the current exit candidate has been active
    exit_dwell_sec: float

    # Short reason describing the current detector decision
    reason: str

    # Evidence source used for stationary-enter decisions
    enter_evidence_source: str


class ZuptDetector:
    """Detects zero-velocity intervals using IMU data."""

    def __init__(self, config: ZuptDetectorConfig) -> None:
        self._config: ZuptDetectorConfig = config
        self._state: ZuptDetectorState = ZuptDetectorState(
            current_zupt_variance_mps2=config.moving_zupt_variance_mps2
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
        """Update the detector using one fused IMU sample.

        Invalid or non-monotonic timestamps are dropped because the
        detector dwell logic is time-based
        """

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

        if not _vector3_is_finite(gyro_vector_rads) or not _vector3_is_finite(
            accel_vector_mps2
        ):
            self._state.last_timestamp_sec = timestamp
            self._state.last_reason = "invalid_imu_sample"
            return self._build_decision(self._state.last_reason)

        gyro_norm_rads: float = _norm3(gyro_vector_rads)
        accel_norm_mps2: float = _norm3(accel_vector_mps2)
        enter_dt_sec: float = _sample_dt_sec(timestamp, previous_timestamp_sec)
        filtered_gyro_norm_rads: float = _ema_update(
            sample=gyro_norm_rads,
            previous=self._state.last_filtered_gyro_norm_rads,
            dt_sec=enter_dt_sec,
            time_constant_sec=self._config.enter_smoothing_time_constant_sec,
        )
        filtered_accel_norm_mps2: float = _ema_update(
            sample=accel_norm_mps2,
            previous=self._state.last_filtered_accel_norm_mps2,
            dt_sec=enter_dt_sec,
            time_constant_sec=self._config.enter_smoothing_time_constant_sec,
        )

        self._state.last_timestamp_sec = timestamp
        self._state.last_gyro_norm_rads = gyro_norm_rads
        self._state.last_accel_norm_mps2 = accel_norm_mps2
        self._state.last_filtered_gyro_norm_rads = filtered_gyro_norm_rads
        self._state.last_filtered_accel_norm_mps2 = filtered_accel_norm_mps2
        self._state.enter_evidence_source = _enter_evidence_source(
            self._config.enter_smoothing_time_constant_sec
        )

        stationary_candidate: bool = (
            filtered_gyro_norm_rads <= self._config.gyro_enter_threshold_rads
            and filtered_accel_norm_mps2 <= self._config.accel_enter_threshold_mps2
        )
        moving_candidate: bool = (
            gyro_norm_rads >= self._config.gyro_exit_threshold_rads
            or accel_norm_mps2 >= self._config.accel_exit_threshold_mps2
        )

        if self._state.stationary:
            self._update_exit_candidate(
                timestamp,
                previous_timestamp_sec,
                moving_candidate,
            )
        else:
            self._update_enter_candidate(timestamp, stationary_candidate)

        self._state.current_zupt_variance_mps2 = self._compute_zupt_variance_mps2()
        return self._build_decision(self._state.last_reason)

    def _update_enter_candidate(
        self, timestamp_sec: float, stationary_candidate: bool
    ) -> None:
        if stationary_candidate:
            if self._state.enter_candidate_start_sec is None:
                self._state.enter_candidate_start_sec = timestamp_sec
                self._state.last_reason = "enter_candidate_started"
                return

            enter_dwell_sec: float = (
                timestamp_sec - self._state.enter_candidate_start_sec
            )
            if _duration_reached(enter_dwell_sec, self._config.min_stationary_sec):
                self._state.stationary = True
                self._state.enter_candidate_start_sec = None
                self._state.exit_candidate_start_sec = None
                self._state.last_reason = (
                    "stationary_asserted_" f"{self._state.enter_evidence_source}"
                )
                return

            self._state.last_reason = (
                "enter_candidate_pending_" f"{self._state.enter_evidence_source}"
            )
            return

        self._state.enter_candidate_start_sec = None
        self._state.last_reason = "moving"

    def _update_exit_candidate(
        self,
        timestamp_sec: float,
        previous_timestamp_sec: Optional[float],
        moving_candidate: bool,
    ) -> None:
        if moving_candidate:
            if self._state.exit_candidate_start_sec is None:
                # The first loud IMU sample represents motion observed over
                # the interval since the previous accepted timestamp
                candidate_start_sec: float = (
                    previous_timestamp_sec
                    if previous_timestamp_sec is not None
                    else timestamp_sec
                )
                self._state.exit_candidate_start_sec = candidate_start_sec

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
            return

        self._state.exit_candidate_start_sec = None
        self._state.last_reason = "stationary_held"

    def _compute_zupt_variance_mps2(self) -> float:
        if not self._state.stationary:
            return float(self._config.moving_zupt_variance_mps2)

        base_variance_mps2: float = self._config.zupt_velocity_sigma_mps**2

        gyro_ratio: float = _normalized_ratio(
            self._state.last_gyro_norm_rads,
            self._config.gyro_exit_threshold_rads,
        )
        accel_ratio: float = _normalized_ratio(
            self._state.last_accel_norm_mps2,
            self._config.accel_exit_threshold_mps2,
        )
        normalized_margin: float = max(gyro_ratio, accel_ratio)

        inflation: float = 1.0 + (
            max(self._config.stationary_variance_inflation - 1.0, 0.0)
            * normalized_margin
        )
        return base_variance_mps2 * inflation

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
            filtered_gyro_norm_rads=self._state.last_filtered_gyro_norm_rads,
            filtered_accel_norm_mps2=self._state.last_filtered_accel_norm_mps2,
            zupt_variance_mps2=self._state.current_zupt_variance_mps2,
            enter_dwell_sec=enter_dwell_sec,
            exit_dwell_sec=exit_dwell_sec,
            reason=reason,
            enter_evidence_source=self._state.enter_evidence_source,
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


def _normalized_ratio(value: float, threshold: float) -> float:
    if not math.isfinite(value) or not math.isfinite(threshold) or threshold <= 0.0:
        return 1.0

    return min(max(value / threshold, 0.0), 1.0)


def _sample_dt_sec(
    timestamp_sec: float, previous_timestamp_sec: Optional[float]
) -> float:
    if previous_timestamp_sec is None:
        return 0.0
    if timestamp_sec <= previous_timestamp_sec:
        return 0.0
    return timestamp_sec - previous_timestamp_sec


def _ema_update(
    *, sample: float, previous: float, dt_sec: float, time_constant_sec: float
) -> float:
    if not math.isfinite(sample):
        return sample

    if (
        not math.isfinite(previous)
        or dt_sec <= 0.0
        or not math.isfinite(time_constant_sec)
        or time_constant_sec <= 0.0
    ):
        return sample

    alpha: float = 1.0 - math.exp(-dt_sec / time_constant_sec)
    return previous + alpha * (sample - previous)


def _enter_evidence_source(time_constant_sec: float) -> str:
    if math.isfinite(time_constant_sec) and time_constant_sec > 0.0:
        return "filtered"
    return "raw"


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
