################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Lightweight 1D forward-velocity estimator with ZUPT updates."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from typing import Optional

import numpy as np
from numpy.typing import NDArray


_FLOAT_ARRAY = NDArray[np.float64]


@dataclass
class VelocityEstimatorConfig:
    """Configuration for the 1D forward velocity estimator."""

    # Process noise for forward velocity random walk, (m/s)^2 per second
    process_noise_v: float = 0.2

    # Process noise for acceleration bias random walk, (m/s^2)^2 per second
    process_noise_b: float = 0.05

    # Initial speed sigma, meters per second
    initial_speed_sigma: float = 0.5

    # Initial acceleration bias sigma, meters per second^2
    initial_bias_sigma: float = 0.2

    # Maximum allowed dt before clamping, seconds
    max_dt_sec: float = 0.2

    # Minimum allowed dt before clamping, seconds
    min_dt_sec: float = 0.0


@dataclass
class VelocityEstimatorCounters:
    """Counters for estimator update activity."""

    # Number of successful ZUPT updates
    zupt_updates: int = 0

    # Number of successful IMU predict updates
    imu_updates: int = 0


@dataclass
class VelocityEstimatorState:
    """Mutable estimator state and diagnostics."""

    # Last update timestamp in seconds, or None before the first update
    last_t: Optional[float] = None

    # Forward velocity estimate in meters per second
    v: float = 0.0

    # Forward acceleration bias estimate in meters per second^2
    b: float = 0.0

    # State covariance matrix in (m/s, m/s^2)^2
    P: _FLOAT_ARRAY = field(default_factory=lambda: np.eye(2, dtype=np.float64))

    # Reason string for the last update decision
    last_reason: str = "init"

    # Last dt used in prediction, seconds
    last_dt_sec: float = 0.0

    # True when dt was clamped to zero for stability
    dt_clamped: bool = False

    # Last ZUPT innovation, meters per second
    last_innovation: float = 0.0

    # Last ZUPT innovation variance
    last_S: float = 0.0

    # Last Kalman gain used in ZUPT update
    last_K: _FLOAT_ARRAY = field(default_factory=lambda: np.zeros(2, dtype=np.float64))

    # Update counters for diagnostics
    counters: VelocityEstimatorCounters = field(
        default_factory=VelocityEstimatorCounters
    )


class VelocityEstimator:
    """Estimate forward velocity using IMU acceleration and ZUPT updates."""

    def __init__(self, config: VelocityEstimatorConfig) -> None:
        self._config: VelocityEstimatorConfig = config
        self._bias_prior_mean: Optional[float] = None
        self._bias_prior_var: Optional[float] = None
        self._state: VelocityEstimatorState = VelocityEstimatorState()
        self.reset()

    @property
    def state(self) -> VelocityEstimatorState:
        """Return the mutable estimator state."""

        return self._state

    def reset(self) -> None:
        """Reset estimator state to the configured prior."""

        speed_sigma: float = float(self._config.initial_speed_sigma)
        bias_sigma: float = float(self._config.initial_bias_sigma)
        prior_bias_mean: float = 0.0
        prior_bias_var: float = bias_sigma * bias_sigma

        if self._bias_prior_mean is not None and self._bias_prior_var is not None:
            if _is_finite(self._bias_prior_mean) and _is_finite(self._bias_prior_var):
                if self._bias_prior_var > 0.0:
                    prior_bias_mean = float(self._bias_prior_mean)
                    prior_bias_var = float(self._bias_prior_var)

        self._state = VelocityEstimatorState(
            last_t=None,
            v=0.0,
            b=prior_bias_mean,
            P=np.array(
                [
                    [speed_sigma * speed_sigma, 0.0],
                    [0.0, prior_bias_var],
                ],
                dtype=np.float64,
            ),
            last_reason="reset",
            last_dt_sec=0.0,
            dt_clamped=False,
            last_innovation=0.0,
            last_S=0.0,
            last_K=np.zeros(2, dtype=np.float64),
            counters=VelocityEstimatorCounters(),
        )

    def set_priors(self, bias_mean: float, bias_var: float) -> None:
        """Set acceleration bias prior mean and variance.

        Args:
            bias_mean: Bias mean in meters per second^2
            bias_var: Bias variance in (meters per second^2)^2
        """

        bias_mean_value: float = float(bias_mean)
        bias_var_value: float = float(bias_var)
        if not _is_finite(bias_mean_value) or not _is_finite(bias_var_value):
            self._state.last_reason = "prior_nonfinite"
            return
        if bias_var_value <= 0.0:
            self._state.last_reason = "prior_nonpositive"
            return

        self._bias_prior_mean = bias_mean_value
        self._bias_prior_var = bias_var_value

        self._state.b = bias_mean_value
        self._state.P[1, 1] = bias_var_value
        self._state.P = _symmetrize(self._state.P)
        self._state.last_reason = "prior_update"

    def predict(
        self,
        timestamp_sec: float,
        a_f_mps2: float,
        a_f_var: Optional[float] = None,
    ) -> dict[str, object]:
        """Propagate the estimator with forward acceleration.

        Args:
            timestamp_sec: Timestamp in seconds
            a_f_mps2: Forward acceleration in meters per second^2
            a_f_var: Optional acceleration variance in (m/s^2)^2
        """

        timestamp: float = float(timestamp_sec)
        accel: float = float(a_f_mps2)

        if not _is_finite(timestamp) or not _is_finite(accel):
            self._state.last_reason = "predict_nonfinite"
            return self._predict_diagnostics(accel, a_f_var)

        dt: float = 0.0
        dt_clamped: bool = False
        if self._state.last_t is None:
            self._state.last_t = timestamp
            self._state.last_dt_sec = 0.0
            self._state.dt_clamped = False
            self._state.last_reason = "predict_first"
            return self._predict_diagnostics(accel, a_f_var)

        dt = timestamp - self._state.last_t
        if dt < self._config.min_dt_sec:
            dt = 0.0
            dt_clamped = True
        if dt > self._config.max_dt_sec:
            dt = 0.0
            dt_clamped = True

        self._state.last_t = timestamp
        self._state.last_dt_sec = dt
        self._state.dt_clamped = dt_clamped

        if dt == 0.0:
            self._state.last_reason = "predict_dt_clamped" if dt_clamped else "predict"
            return self._predict_diagnostics(accel, a_f_var)

        v_prev: float = float(self._state.v)
        b_prev: float = float(self._state.b)

        f11: float = 1.0
        f12: float = -dt
        f21: float = 0.0
        f22: float = 1.0

        self._state.v = v_prev + (accel - b_prev) * dt
        self._state.b = b_prev

        q_v: float = float(self._config.process_noise_v)
        q_b: float = float(self._config.process_noise_b)

        # (m/s)^2 per second, continuous-time spectral density approximation
        q11: float = q_v * dt

        # (m/s^2)^2 per second, continuous-time spectral density approximation
        q22: float = q_b * dt

        p: _FLOAT_ARRAY = self._state.P
        p00: float = float(p[0, 0])
        p01: float = float(p[0, 1])
        p10: float = float(p[1, 0])
        p11: float = float(p[1, 1])

        fp00: float = f11 * p00 + f12 * p10
        fp01: float = f11 * p01 + f12 * p11
        fp10: float = f21 * p00 + f22 * p10
        fp11: float = f21 * p01 + f22 * p11

        p00_new: float = fp00 * f11 + fp01 * f12 + q11
        p01_new: float = fp00 * f21 + fp01 * f22
        p10_new: float = fp10 * f11 + fp11 * f12
        p11_new: float = fp10 * f21 + fp11 * f22 + q22

        self._state.P = np.array(
            [[p00_new, p01_new], [p10_new, p11_new]], dtype=np.float64
        )
        self._state.P = _symmetrize(self._state.P)
        self._state.last_reason = "predict"
        self._state.counters.imu_updates += 1

        return self._predict_diagnostics(accel, a_f_var)

    def update_zupt(self, timestamp_sec: float, zupt_var: float) -> dict[str, object]:
        """Apply a zero-velocity update measurement.

        Args:
            timestamp_sec: Timestamp in seconds
            zupt_var: Measurement variance in (m/s)^2
        """

        timestamp: float = float(timestamp_sec)
        zupt_var_value: float = float(zupt_var)

        if not _is_finite(timestamp) or not _is_finite(zupt_var_value):
            self._state.last_reason = "zupt_nonfinite"
            return self._zupt_diagnostics(zupt_var_value)

        self._state.last_t = timestamp
        self._state.last_dt_sec = 0.0
        self._state.dt_clamped = False

        # (m/s)^2, variance floor to keep innovation variance positive
        zupt_var_floor: float = 1e-12
        if zupt_var_value < zupt_var_floor:
            zupt_var_value = zupt_var_floor

        p: _FLOAT_ARRAY = self._state.P
        p_vv: float = float(p[0, 0])
        p_bv: float = float(p[1, 0])
        s: float = p_vv + zupt_var_value

        if not _is_finite(s) or s <= 0.0:
            self._state.last_reason = "zupt_invalid_s"
            return self._zupt_diagnostics(zupt_var_value)

        v_pred: float = float(self._state.v)
        innovation: float = 0.0 - v_pred

        k0: float = p_vv / s
        k1: float = p_bv / s

        self._state.v = v_pred + k0 * innovation
        self._state.b = float(self._state.b) + k1 * innovation

        i00: float = 1.0 - k0
        i01: float = 0.0
        i10: float = -k1
        i11: float = 1.0

        p00: float = float(p[0, 0])
        p01: float = float(p[0, 1])
        p10: float = float(p[1, 0])
        p11: float = float(p[1, 1])

        ik00: float = i00 * p00 + i01 * p10
        ik01: float = i00 * p01 + i01 * p11
        ik10: float = i10 * p00 + i11 * p10
        ik11: float = i10 * p01 + i11 * p11

        p00_new: float = ik00 * i00 + ik01 * i01 + k0 * zupt_var_value * k0
        p01_new: float = ik00 * i10 + ik01 * i11 + k0 * zupt_var_value * k1
        p10_new: float = ik10 * i00 + ik11 * i01 + k1 * zupt_var_value * k0
        p11_new: float = ik10 * i10 + ik11 * i11 + k1 * zupt_var_value * k1

        self._state.P = np.array(
            [[p00_new, p01_new], [p10_new, p11_new]], dtype=np.float64
        )
        self._state.P = _symmetrize(self._state.P)
        self._state.last_innovation = innovation
        self._state.last_S = s
        self._state.last_K = np.array([k0, k1], dtype=np.float64)
        self._state.last_reason = "zupt"
        self._state.counters.zupt_updates += 1

        return self._zupt_diagnostics(zupt_var_value)

    def get_velocity_mps(self) -> float:
        """Return the estimated forward velocity in meters per second."""

        return float(self._state.v)

    def get_bias_mps2(self) -> float:
        """Return the estimated acceleration bias in meters per second^2."""

        return float(self._state.b)

    def get_covariance(self) -> _FLOAT_ARRAY:
        """Return a copy of the 2x2 state covariance matrix."""

        return self._state.P.copy()

    def get_diagnostics_snapshot(self) -> dict[str, object]:
        """Return a snapshot of internal diagnostics."""

        p: _FLOAT_ARRAY = self._state.P
        return {
            "last_t": self._state.last_t,
            "v": float(self._state.v),
            "b": float(self._state.b),
            "P_vv": float(p[0, 0]),
            "P_vb": float(p[0, 1]),
            "P_bb": float(p[1, 1]),
            "last_reason": self._state.last_reason,
            "last_dt_sec": float(self._state.last_dt_sec),
            "dt_clamped": bool(self._state.dt_clamped),
            "last_innovation": float(self._state.last_innovation),
            "last_S": float(self._state.last_S),
            "last_K": self._state.last_K.copy(),
            "zupt_updates": self._state.counters.zupt_updates,
            "imu_updates": self._state.counters.imu_updates,
        }

    def _predict_diagnostics(
        self, accel: float, a_f_var: Optional[float]
    ) -> dict[str, object]:
        p: _FLOAT_ARRAY = self._state.P
        diagnostics: dict[str, object] = {
            "dt_sec": float(self._state.last_dt_sec),
            "dt_clamped": bool(self._state.dt_clamped),
            "v": float(self._state.v),
            "b": float(self._state.b),
            "P_vv": float(p[0, 0]),
            "P_bb": float(p[1, 1]),
            "a_f": float(accel),
        }
        if a_f_var is not None:
            diagnostics["a_f_var"] = float(a_f_var)
        return diagnostics

    def _zupt_diagnostics(self, zupt_var: float) -> dict[str, object]:
        p: _FLOAT_ARRAY = self._state.P
        return {
            "y": float(self._state.last_innovation),
            "S": float(self._state.last_S),
            "K": self._state.last_K.copy(),
            "zupt_var": float(zupt_var),
            "v": float(self._state.v),
            "b": float(self._state.b),
            "P_vv": float(p[0, 0]),
            "P_bb": float(p[1, 1]),
        }


def _is_finite(value: float) -> bool:
    return bool(np.isfinite(value))


def _symmetrize(mat: _FLOAT_ARRAY) -> _FLOAT_ARRAY:
    return 0.5 * (mat + mat.T)


def run_velocity_estimator_self_test() -> dict[str, object]:
    """Run a minimal sanity check for basic predict/update behavior."""

    config: VelocityEstimatorConfig = VelocityEstimatorConfig(
        process_noise_v=0.1,
        process_noise_b=0.01,
        initial_speed_sigma=0.1,
        initial_bias_sigma=0.1,
    )
    estimator: VelocityEstimator = VelocityEstimator(config)

    estimator.predict(0.0, 1.0)
    estimator.predict(0.1, 1.0)
    estimator.update_zupt(0.1, 1e-3)

    return estimator.get_diagnostics_snapshot()
