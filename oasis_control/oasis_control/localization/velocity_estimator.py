################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Forward velocity estimator for 1D IMU propagation with ZUPT updates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from dataclasses import field
from typing import Optional

import numpy as np
from numpy.typing import NDArray


_FLOAT_ARRAY = NDArray[np.float64]


@dataclass
class VelocityEstimatorConfig:
    """Configuration values for the velocity estimator."""

    # Forward speed process noise in (m/s)^2 per second
    process_noise_v: float = 0.4

    # Bias process noise in (m/s^2)^2 per second
    process_noise_b: float = 0.04

    # Initial forward speed sigma in m/s
    initial_speed_sigma: float = 0.5

    # Initial acceleration bias sigma in m/s^2
    initial_bias_sigma: float = 0.2

    # Optional initial bias mean in m/s^2
    initial_bias_mean: Optional[float] = None

    # Optional initial bias variance in (m/s^2)^2
    initial_bias_var: Optional[float] = None

    # Maximum allowed dt in seconds before clamping to zero
    max_dt_sec: float = 0.2

    # Minimum allowed dt in seconds before clamping to zero
    min_dt_sec: float = 0.0

    # Minimum ZUPT variance to avoid divide-by-zero
    zupt_var_floor: float = 1e-12


@dataclass
class VelocityEstimatorCounters:
    """Counters tracking how many updates have been applied."""

    # Count of IMU prediction calls
    imu_updates: int = 0

    # Count of ZUPT measurement updates
    zupt_updates: int = 0


@dataclass
class VelocityEstimatorState:
    """Mutable state for the velocity estimator."""

    # Last IMU timestamp in seconds, or None before the first predict
    last_t: Optional[float] = None

    # Estimated forward speed in m/s
    v: float = 0.0

    # Estimated forward acceleration bias in m/s^2
    b: float = 0.0

    # State covariance matrix for [v, b] in (m/s)^2 and (m/s^2)^2
    P: _FLOAT_ARRAY = field(default_factory=lambda: np.zeros((2, 2), dtype=np.float64))

    # Reason string describing the last update decision
    last_reason: str = "init"

    # Last dt in seconds used by predict
    last_dt_sec: float = 0.0

    # True when dt was clamped to zero
    dt_clamped: bool = False

    # Last ZUPT innovation value in m/s
    last_innovation: float = 0.0

    # Last ZUPT innovation variance
    last_S: float = 0.0

    # Last ZUPT Kalman gain vector
    last_K: _FLOAT_ARRAY = field(default_factory=lambda: np.zeros(2, dtype=np.float64))

    # Counters for predict and update calls
    counters: VelocityEstimatorCounters = field(
        default_factory=VelocityEstimatorCounters
    )


class VelocityEstimator:
    """Kalman-based forward velocity estimator with ZUPT corrections."""

    def __init__(self, config: VelocityEstimatorConfig) -> None:
        self._config: VelocityEstimatorConfig = config
        self._state: VelocityEstimatorState = VelocityEstimatorState()
        self.reset()

    @property
    def state(self) -> VelocityEstimatorState:
        """Return the mutable estimator state."""

        return self._state

    def reset(self) -> None:
        """Reset the estimator state to configured priors."""

        initial_speed_var: float = self._config.initial_speed_sigma**2
        initial_bias_var: float = self._config.initial_bias_sigma**2
        P: _FLOAT_ARRAY = np.diag([initial_speed_var, initial_bias_var]).astype(
            np.float64
        )

        v_init: float = 0.0
        b_init: float = 0.0

        if self._config.initial_bias_mean is not None:
            bias_mean: float = float(self._config.initial_bias_mean)
            if _is_finite(bias_mean):
                b_init = bias_mean
        if self._config.initial_bias_var is not None:
            bias_var: float = float(self._config.initial_bias_var)
            if _is_finite(bias_var) and bias_var > 0.0:
                P[1, 1] = bias_var

        self._state = VelocityEstimatorState(
            last_t=None,
            v=v_init,
            b=b_init,
            P=P,
            last_reason="reset",
            last_dt_sec=0.0,
            dt_clamped=False,
            last_innovation=0.0,
            last_S=0.0,
            last_K=np.zeros(2, dtype=np.float64),
            counters=VelocityEstimatorCounters(),
        )

    def set_priors(self, bias_mean: float, bias_var: float) -> None:
        """Set the bias prior mean and variance.

        Args:
            bias_mean: Bias mean in m/s^2
            bias_var: Bias variance in (m/s^2)^2
        """

        bias_mean_val: float = float(bias_mean)
        bias_var_val: float = float(bias_var)
        if not _is_finite(bias_mean_val) or not _is_finite(bias_var_val):
            self._state.last_reason = "priors_non_finite"
            return
        if bias_var_val <= 0.0:
            self._state.last_reason = "priors_non_positive"
            return

        self._state.b = bias_mean_val
        self._state.P[1, 1] = bias_var_val
        self._state.P = _symmetrize(self._state.P)
        self._state.last_reason = "priors_updated"

    def predict(
        self,
        timestamp_sec: float,
        a_f_mps2: float,
        a_f_var: Optional[float] = None,
    ) -> dict[str, object]:
        """Propagate the state using forward acceleration.

        Args:
            timestamp_sec: Timestamp in seconds
            a_f_mps2: Forward acceleration measurement in m/s^2
            a_f_var: Optional acceleration variance in (m/s^2)^2
        """

        timestamp: float = float(timestamp_sec)
        accel: float = float(a_f_mps2)
        if not _is_finite(timestamp) or not _is_finite(accel):
            self._state.last_reason = "predict_non_finite"
            return self._predict_diagnostics(0.0, False, accel, a_f_var)

        dt: float = 0.0
        dt_clamped: bool = False
        if self._state.last_t is None:
            self._state.last_t = timestamp
            self._state.last_reason = "predict_init"
            self._state.last_dt_sec = 0.0
            self._state.dt_clamped = False
            return self._predict_diagnostics(0.0, False, accel, a_f_var)

        dt = timestamp - self._state.last_t
        dt, dt_clamped = _clamp_dt(
            dt,
            self._config.min_dt_sec,
            self._config.max_dt_sec,
        )
        self._state.last_t = timestamp

        F: _FLOAT_ARRAY = np.array(
            [
                [1.0, -dt],
                [0.0, 1.0],
            ],
            dtype=np.float64,
        )
        x: _FLOAT_ARRAY = np.array([self._state.v, self._state.b], dtype=np.float64)

        x = F @ x + np.array([accel * dt, 0.0], dtype=np.float64)

        # Process noise uses continuous-time spectral densities scaled by dt
        q_v: float = float(self._config.process_noise_v)
        q_b: float = float(self._config.process_noise_b)
        Q: _FLOAT_ARRAY = np.array(
            [
                [q_v * dt, 0.0],
                [0.0, q_b * dt],
            ],
            dtype=np.float64,
        )

        P: _FLOAT_ARRAY = F @ self._state.P @ F.T + Q

        self._state.v = float(x[0])
        self._state.b = float(x[1])
        self._state.P = _symmetrize(P)
        self._state.last_reason = "predict"
        self._state.last_dt_sec = dt
        self._state.dt_clamped = dt_clamped
        self._state.counters.imu_updates += 1

        return self._predict_diagnostics(dt, dt_clamped, accel, a_f_var)

    def update_zupt(self, timestamp_sec: float, zupt_var: float) -> dict[str, object]:
        """Apply a zero-velocity measurement update.

        Args:
            timestamp_sec: Timestamp in seconds
            zupt_var: ZUPT variance in (m/s)^2
        """

        timestamp: float = float(timestamp_sec)
        if not _is_finite(timestamp):
            self._state.last_reason = "zupt_non_finite_time"
            return self._zupt_diagnostics(
                0.0,
                0.0,
                np.zeros(2, dtype=np.float64),
                self._config.zupt_var_floor,
            )

        zupt_var_val: float = float(zupt_var)
        if not _is_finite(zupt_var_val):
            self._state.last_reason = "zupt_non_finite_var"
            return self._zupt_diagnostics(
                0.0,
                0.0,
                np.zeros(2, dtype=np.float64),
                self._config.zupt_var_floor,
            )

        zupt_var_val = max(zupt_var_val, self._config.zupt_var_floor)

        y: float = -self._state.v
        S: float = float(self._state.P[0, 0] + zupt_var_val)

        if not _is_finite(S) or S <= 0.0:
            self._state.last_reason = "zupt_bad_innovation"
            return self._zupt_diagnostics(
                y,
                S,
                np.zeros(2, dtype=np.float64),
                zupt_var_val,
            )

        K: _FLOAT_ARRAY = (self._state.P[:, 0] / S).astype(np.float64)

        x: _FLOAT_ARRAY = np.array([self._state.v, self._state.b], dtype=np.float64)
        x = x + K * y

        H: _FLOAT_ARRAY = np.array([[1.0, 0.0]], dtype=np.float64)
        I: _FLOAT_ARRAY = np.eye(2, dtype=np.float64)
        KH: _FLOAT_ARRAY = K[:, None] @ H
        P: _FLOAT_ARRAY = (I - KH) @ self._state.P @ (I - KH).T
        P = P + np.outer(K, K) * zupt_var_val

        self._state.v = float(x[0])
        self._state.b = float(x[1])
        self._state.P = _symmetrize(P)
        self._state.last_reason = "zupt_update"
        self._state.last_innovation = y
        self._state.last_S = S
        self._state.last_K = K
        self._state.counters.zupt_updates += 1

        return self._zupt_diagnostics(y, S, K, zupt_var_val)

    def get_velocity_mps(self) -> float:
        """Return the estimated forward speed in m/s."""

        return float(self._state.v)

    def get_bias_mps2(self) -> float:
        """Return the estimated acceleration bias in m/s^2."""

        return float(self._state.b)

    def get_covariance(self) -> _FLOAT_ARRAY:
        """Return a copy of the 2x2 state covariance matrix."""

        return np.array(self._state.P, dtype=np.float64)

    def get_diagnostics_snapshot(self) -> dict[str, object]:
        """Return a snapshot of recent diagnostic values."""

        return {
            "last_t": self._state.last_t,
            "v_mps": self._state.v,
            "b_mps2": self._state.b,
            "P_vv": float(self._state.P[0, 0]),
            "P_bb": float(self._state.P[1, 1]),
            "last_reason": self._state.last_reason,
            "last_dt_sec": self._state.last_dt_sec,
            "dt_clamped": self._state.dt_clamped,
            "last_innovation": self._state.last_innovation,
            "last_S": self._state.last_S,
            "last_K": np.array(self._state.last_K, dtype=np.float64),
            "imu_updates": self._state.counters.imu_updates,
            "zupt_updates": self._state.counters.zupt_updates,
        }

    def _predict_diagnostics(
        self,
        dt: float,
        dt_clamped: bool,
        accel: float,
        a_f_var: Optional[float],
    ) -> dict[str, object]:
        a_f_var_used: Optional[float] = None
        if a_f_var is not None:
            a_f_var_val: float = float(a_f_var)
            if _is_finite(a_f_var_val):
                a_f_var_used = a_f_var_val
        return {
            "dt_sec": dt,
            "dt_clamped": dt_clamped,
            "v_mps": self._state.v,
            "b_mps2": self._state.b,
            "P_vv": float(self._state.P[0, 0]),
            "P_bb": float(self._state.P[1, 1]),
            "a_f_mps2": accel,
            "a_f_var": a_f_var_used,
        }

    def _zupt_diagnostics(
        self,
        innovation: float,
        S: float,
        K: _FLOAT_ARRAY,
        zupt_var: float,
    ) -> dict[str, object]:
        return {
            "innovation": innovation,
            "S": S,
            "K": np.array(K, dtype=np.float64),
            "zupt_var": zupt_var,
            "v_mps": self._state.v,
            "b_mps2": self._state.b,
            "P_vv": float(self._state.P[0, 0]),
            "P_bb": float(self._state.P[1, 1]),
        }


def run_velocity_estimator_self_test() -> dict[str, object]:
    """Run a minimal self-test sequence and return diagnostics."""

    config: VelocityEstimatorConfig = VelocityEstimatorConfig(
        process_noise_v=0.0,
        process_noise_b=0.0,
        initial_speed_sigma=0.1,
        initial_bias_sigma=0.1,
    )
    estimator: VelocityEstimator = VelocityEstimator(config)

    estimator.predict(0.0, 0.0)
    estimator.predict(0.1, 1.0)
    zupt_diag: dict[str, object] = estimator.update_zupt(0.1, 0.01)

    snapshot: dict[str, object] = estimator.get_diagnostics_snapshot()

    return {
        "zupt": zupt_diag,
        "snapshot": snapshot,
    }


def _clamp_dt(dt: float, min_dt: float, max_dt: float) -> tuple[float, bool]:
    if dt < min_dt:
        return 0.0, True
    if dt > max_dt:
        return 0.0, True
    return dt, False


def _is_finite(value: float) -> bool:
    return math.isfinite(value)


def _symmetrize(matrix: _FLOAT_ARRAY) -> _FLOAT_ARRAY:
    return 0.5 * (matrix + matrix.T)
