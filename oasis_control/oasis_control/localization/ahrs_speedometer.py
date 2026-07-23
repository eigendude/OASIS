################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS-independent longitudinal speed estimation with full ZUPT updates."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from typing import TypeAlias

import numpy as np


FloatArray: TypeAlias = np.ndarray

# Negative eigenvalues at this scale are treated as eigensolver roundoff only
COVARIANCE_TOLERANCE: float = 1.0e-10
NANOSECONDS_PER_SECOND: int = 1_000_000_000


@dataclass(frozen=True)
class AhrsSpeedometerConfig:
    """Estimator settings with variances expressed in SI squared units.

    ``initial_speed_variance_mps2`` is the uncertainty assigned before the
    first stationary update. ``fallback_accel_variance_mps4`` is used when the
    IMU does not publish acceleration covariance. ``fallback_angular_variance``
    is used independently on each angular axis when gyro covariance is absent.
    ``speed_process_noise_mps2_per_sec`` is continuous speed variance growth.
    ``max_interval_sec`` limits integration over missing IMU data, and
    ``zupt_freshness_sec`` limits a stationary observation's age or lead. Both
    time limits are converted once to exact integer nanosecond thresholds.
    """

    initial_speed_variance_mps2: float = 1.0
    fallback_accel_variance_mps4: float = 4.0
    fallback_angular_variance_rads2: float = 1.0
    speed_process_noise_mps2_per_sec: float = 0.01
    max_interval_sec: float = 0.25
    zupt_freshness_sec: float = 0.1
    _max_interval_ns: int = field(init=False, repr=False)
    _zupt_freshness_ns: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """Validate configuration ranges."""

        values: tuple[float, ...] = (
            self.initial_speed_variance_mps2,
            self.fallback_accel_variance_mps4,
            self.fallback_angular_variance_rads2,
            self.speed_process_noise_mps2_per_sec,
        )
        if not all(np.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError(
                "variances and process noise must be finite and nonnegative"
            )
        if not np.isfinite(self.max_interval_sec) or self.max_interval_sec <= 0.0:
            raise ValueError("maximum interval must be finite and positive")
        max_interval_ns: int = round(self.max_interval_sec * NANOSECONDS_PER_SECOND)
        if max_interval_ns <= 0:
            raise ValueError("maximum interval must be at least one nanosecond")
        if not np.isfinite(self.zupt_freshness_sec) or self.zupt_freshness_sec < 0.0:
            raise ValueError("ZUPT freshness must be finite and nonnegative")
        zupt_freshness_ns: int = round(self.zupt_freshness_sec * NANOSECONDS_PER_SECOND)
        object.__setattr__(self, "_max_interval_ns", max_interval_ns)
        object.__setattr__(self, "_zupt_freshness_ns", zupt_freshness_ns)

    @property
    def max_interval_ns(self) -> int:
        """Return the maximum accepted interval in integer nanoseconds."""

        return self._max_interval_ns

    @property
    def zupt_freshness_ns(self) -> int:
        """Return the ZUPT freshness threshold in integer nanoseconds."""

        return self._zupt_freshness_ns


@dataclass(frozen=True)
class AhrsImuSample:
    """Validated mounted AHRS sample supplied to the estimator.

    ``timestamp_ns`` is the exact measurement time in nanoseconds.
    ``accel_x_mps2`` is gravity-free acceleration along ``base_link +x``.
    ``angular_velocity_rads`` contains mounted x/y/z rates in rad/s.
    ``accel_x_variance_mps4`` is ``None`` when unavailable.
    ``angular_covariance_rads2`` is a complete 3-by-3 matrix or ``None``.
    """

    timestamp_ns: int
    accel_x_mps2: float
    angular_velocity_rads: FloatArray
    accel_x_variance_mps4: float | None
    angular_covariance_rads2: FloatArray | None


@dataclass(frozen=True)
class StationaryTwistObservation:
    """A validated six-dimensional stationary-twist observation.

    ``timestamp_ns`` is the exact observation time in nanoseconds. ``mean``
    follows ROS twist ordering and contains SI linear and angular rates.
    ``covariance`` is its complete row-major 6-by-6 covariance. Together with
    the prior it must produce a finite, symmetric, positive-definite innovation
    covariance so the correction has a unique stable solve.
    """

    timestamp_ns: int
    mean: FloatArray
    covariance: FloatArray


@dataclass(frozen=True)
class SpeedometerEstimate:
    """Current estimate ready for conversion to a ROS twist message.

    ``timestamp_ns`` is the exact state time in nanoseconds. ``mean`` follows
    standard six-axis ROS twist ordering. ``covariance`` is the complete
    row-major 6-by-6 covariance. ``initialized`` means an active, fresh ZUPT
    established zero-speed trust. ``zupt_applied`` reports whether this update
    consumed a new observation. ``used_accel_covariance_fallback`` reports
    conservative acceleration variance fallback.
    ``used_angular_covariance_fallback`` reports conservative angular-rate
    variance fallback.
    """

    timestamp_ns: int
    mean: FloatArray
    covariance: FloatArray
    initialized: bool
    zupt_applied: bool
    used_accel_covariance_fallback: bool
    used_angular_covariance_fallback: bool


class AhrsSpeedometer:
    """Estimate signed forward speed and correct its 4D substate with ZUPT."""

    _G: FloatArray = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    def __init__(self, config: AhrsSpeedometerConfig) -> None:
        self._config: AhrsSpeedometerConfig = config
        self._state: FloatArray = np.zeros(4, dtype=np.float64)
        self._covariance: FloatArray = np.zeros((4, 4), dtype=np.float64)
        self._covariance[0, 0] = config.initial_speed_variance_mps2
        self._last_timestamp_ns: int | None = None
        self._initialized: bool = False
        self._zupt_active: bool = False
        self._latest_zupt: StationaryTwistObservation | None = None
        self._last_applied_zupt_ns: int | None = None
        self.last_rejection: str | None = None
        self.last_zupt_status: str = "none"

    @property
    def initialized(self) -> bool:
        """Return whether a stationary observation initialized speed."""

        return self._initialized

    def set_zupt_active(self, active: bool) -> None:
        """Set the independently received stationarity flag."""

        self._zupt_active = active

    def store_zupt(self, observation: StationaryTwistObservation) -> bool:
        """Store a newer validated observation and report whether accepted."""

        if self._latest_zupt is not None:
            if observation.timestamp_ns == self._latest_zupt.timestamp_ns:
                self.last_zupt_status = "duplicate"
                return False
            if observation.timestamp_ns < self._latest_zupt.timestamp_ns:
                self.last_zupt_status = "out_of_order"
                return False
        self._latest_zupt = observation
        self.last_zupt_status = "accepted"
        return True

    def update(self, sample: AhrsImuSample) -> SpeedometerEstimate | None:
        """Consume an IMU sample, returning ``None`` when policy rejects it."""

        self.last_rejection = None
        candidate_state: FloatArray = self._state.copy()
        candidate_covariance: FloatArray = self._covariance.copy()
        if self._last_timestamp_ns is not None:
            dt_ns: int = sample.timestamp_ns - self._last_timestamp_ns
            if dt_ns <= 0:
                self.last_rejection = "out_of_order"
                return None
            if dt_ns > self._config.max_interval_ns:
                self._last_timestamp_ns = sample.timestamp_ns
                self.last_rejection = "excessive_gap"
                return None
            if self._initialized:
                dt_sec: float = dt_ns * 1.0e-9
                candidate_state[0] += sample.accel_x_mps2 * dt_sec
                accel_variance_mps4: float = (
                    self._config.fallback_accel_variance_mps4
                    if sample.accel_x_variance_mps4 is None
                    else sample.accel_x_variance_mps4
                )
                candidate_covariance[0, 0] += (
                    dt_sec * dt_sec * accel_variance_mps4
                    + self._config.speed_process_noise_mps2_per_sec * dt_sec
                )

        candidate_state[1:4] = sample.angular_velocity_rads
        if sample.angular_covariance_rads2 is None:
            candidate_covariance[1:4, 1:4] = np.eye(3, dtype=np.float64) * (
                self._config.fallback_angular_variance_rads2
            )
        else:
            candidate_covariance[1:4, 1:4] = sample.angular_covariance_rads2

        # The new angular observation is independent of the previous one
        candidate_covariance[0, 1:4] = 0.0
        candidate_covariance[1:4, 0] = 0.0
        candidate_covariance = _symmetrize(candidate_covariance)
        if not _valid_state_and_covariance(candidate_state, candidate_covariance):
            self.last_rejection = "invalid_propagation"
            return None

        self._last_timestamp_ns = sample.timestamp_ns
        self._state = candidate_state
        self._covariance = candidate_covariance
        zupt_applied: bool = self._apply_pending_zupt(sample.timestamp_ns)

        twist_mean: FloatArray = self._G @ self._state
        twist_covariance: FloatArray = _symmetrize(
            self._G @ self._covariance @ self._G.T
        )
        if not _valid_state_and_covariance(twist_mean, twist_covariance):
            self.last_rejection = "invalid_output"
            return None
        return SpeedometerEstimate(
            timestamp_ns=sample.timestamp_ns,
            mean=twist_mean,
            covariance=twist_covariance,
            initialized=self._initialized,
            zupt_applied=zupt_applied,
            used_accel_covariance_fallback=(sample.accel_x_variance_mps4 is None),
            used_angular_covariance_fallback=(sample.angular_covariance_rads2 is None),
        )

    def _apply_pending_zupt(self, timestamp_ns: int) -> bool:
        observation: StationaryTwistObservation | None = self._latest_zupt
        if not self._zupt_active or observation is None:
            self.last_zupt_status = "inactive"
            return False
        if observation.timestamp_ns == self._last_applied_zupt_ns:
            self.last_zupt_status = "duplicate"
            return False
        if (
            abs(timestamp_ns - observation.timestamp_ns)
            > self._config.zupt_freshness_ns
        ):
            self.last_zupt_status = "stale"
            return False

        prior_state: FloatArray = self._state.copy()
        prior_covariance: FloatArray = self._covariance.copy()
        innovation: FloatArray = observation.mean - self._G @ prior_state
        innovation_covariance: FloatArray = (
            self._G @ prior_covariance @ self._G.T + observation.covariance
        )
        try:
            # Valid ZUPTs must produce a finite, symmetric, positive-definite S
            # Solve S X = H P, then transpose to obtain P H^T S^-1
            kalman_gain: FloatArray = np.transpose(
                np.linalg.solve(innovation_covariance, self._G @ prior_covariance)
            )
        except np.linalg.LinAlgError:
            self.last_zupt_status = "singular_innovation"
            return False

        posterior_state: FloatArray = prior_state + kalman_gain @ innovation
        identity: FloatArray = np.eye(4, dtype=np.float64)
        residual: FloatArray = identity - kalman_gain @ self._G
        posterior_covariance: FloatArray = (
            residual @ prior_covariance @ residual.T
            + kalman_gain @ observation.covariance @ kalman_gain.T
        )
        posterior_covariance = _symmetrize(posterior_covariance)
        if not _valid_state_and_covariance(posterior_state, posterior_covariance):
            self.last_zupt_status = "invalid_posterior"
            return False

        self._state = posterior_state
        self._covariance = posterior_covariance
        self._initialized = True
        self._last_applied_zupt_ns = observation.timestamp_ns
        self.last_zupt_status = "applied"
        return True


def _symmetrize(covariance: FloatArray) -> FloatArray:
    return 0.5 * (covariance + covariance.T)


def _valid_state_and_covariance(state: FloatArray, covariance: FloatArray) -> bool:
    """Validate finite state and positive-semidefinite covariance."""

    if not np.all(np.isfinite(state)) or not np.all(np.isfinite(covariance)):
        return False
    if np.min(np.diag(covariance)) < -COVARIANCE_TOLERANCE:
        return False
    try:
        eigenvalues: FloatArray = np.linalg.eigvalsh(covariance)
    except np.linalg.LinAlgError:
        return False
    return bool(np.min(eigenvalues) >= -COVARIANCE_TOLERANCE)
