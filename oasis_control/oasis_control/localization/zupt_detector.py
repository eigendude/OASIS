################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Zero-velocity update (ZUPT) detector for tilt gyro data."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional
from typing import cast

import numpy as np
from numpy.typing import NDArray


_FLOAT_ARRAY = NDArray[np.float64]


@dataclass
class ZuptDetectorConfig:
    """Configuration values for the ZUPT detector."""

    # Minimum quiet time to enter stationary, in seconds
    min_stationary_sec: float = 0.7

    # Minimum loud time to exit stationary, in seconds
    min_exit_sec: float = 0.15

    # Chi-square gate to enter stationary for df=3
    gate_enter_chi2: float = 7.8147279

    # Chi-square gate to exit stationary for df=3
    gate_exit_chi2: float = 11.3448667

    # Norm gate to enter stationary when covariance inversion fails, rad/s
    omega_norm_enter: float = 0.05

    # Norm gate to exit stationary when covariance inversion fails, rad/s
    omega_norm_exit: float = 0.10

    # Require duty cycle to be exactly zero to gate stationary
    duty_requires_exact_zero: bool = True

    # Tolerance for duty cycle to be treated as zero
    duty_zero_epsilon: float = 0.0

    # Enable adaptive gates based on EWMA of quietness
    use_adaptive_gates: bool = False

    # EWMA smoothing factor for adaptive gate updates
    adaptive_ewma_alpha: float = 0.02

    # Minimum adaptive gate scale relative to nominal
    adaptive_gate_scale_min: float = 0.75

    # Maximum adaptive gate scale relative to nominal
    adaptive_gate_scale_max: float = 2.0

    # ZUPT velocity sigma at entry, meters per second
    zupt_sigma_start_mps: float = 0.10

    # ZUPT velocity sigma after long dwell, meters per second
    zupt_sigma_min_mps: float = 0.01

    # ZUPT variance annealing time constant, seconds
    zupt_anneal_tau_sec: float = 0.7

    # Regularization epsilon added to covariance for inversion
    covariance_regularization_eps: float = 1e-9


@dataclass
class ZuptDetectorState:
    """Mutable state for the ZUPT detector."""

    # Last update timestamp in seconds, or None before the first update
    last_t: Optional[float] = None

    # Last duty cycle value, unitless
    duty_cycle: float = 0.0

    # True when the detector believes the platform is stationary
    stationary: bool = False

    # Accumulated quiet time while seeking stationary, seconds
    candidate_quiet_time: float = 0.0

    # Accumulated loud time while seeking exit, seconds
    candidate_loud_time: float = 0.0

    # Dwell time in the stationary state, seconds
    stationary_dwell_time: float = 0.0

    # Current adaptive scaling factor for gate thresholds
    adaptive_gate_scale: float = 1.0

    # Last Mahalanobis distance squared value
    last_d2: float = 0.0

    # Last raw omega norm, rad/s
    last_omega_norm: float = 0.0

    # Last bias-corrected omega norm, rad/s
    last_omega_c_norm: float = 0.0

    # Last enter gate value
    last_gate_enter: float = 0.0

    # Last exit gate value
    last_gate_exit: float = 0.0

    # Reason string for the last decision
    last_reason: str = "init"


class ZuptDetector:
    """Detects zero-velocity intervals using duty gating and gyro quietness."""

    def __init__(self, config: ZuptDetectorConfig) -> None:
        self._config: ZuptDetectorConfig = config
        self._state: ZuptDetectorState = ZuptDetectorState()
        self._gyro_bias: _FLOAT_ARRAY = np.zeros(3, dtype=np.float64)
        self._gyro_bias_cov: _FLOAT_ARRAY = np.zeros((3, 3), dtype=np.float64)
        self._adaptive_d2_ewma: Optional[float] = None

    @property
    def state(self) -> ZuptDetectorState:
        """Return the mutable detector state."""

        return self._state

    def reset(self) -> None:
        """Reset the detector state and cached adaptive statistics."""

        self._state = ZuptDetectorState()
        self._adaptive_d2_ewma = None

    def set_gyro_bias(self, bias: _FLOAT_ARRAY, cov: _FLOAT_ARRAY) -> None:
        """Set gyro bias mean and covariance.

        Args:
            bias: Gyro bias vector in rad/s
            cov: Gyro bias covariance matrix in (rad/s)^2
        """

        self._gyro_bias = _as_vector(bias, "gyro_bias")
        self._gyro_bias_cov = _as_matrix(cov, "gyro_bias_cov")

    def set_duty_cycle(self, duty: float) -> None:
        """Update the duty cycle used for commanded-stop gating."""

        self._state.duty_cycle = float(duty)

    def update(
        self,
        timestamp_sec: float,
        omega: _FLOAT_ARRAY,
        omega_cov: _FLOAT_ARRAY,
    ) -> dict[str, object]:
        """Update the detector and return the current decision.

        Args:
            timestamp_sec: Timestamp in seconds
            omega: Angular velocity vector in rad/s
            omega_cov: Angular velocity covariance in (rad/s)^2
        """

        timestamp: float = float(timestamp_sec)
        dt: float = 0.0
        dt_clamped: bool = False
        if self._state.last_t is not None:
            dt = timestamp - self._state.last_t
            if dt < 0.0:
                dt = 0.0
                dt_clamped = True
            if dt > 0.2:
                dt = 0.0
                dt_clamped = True
        self._state.last_t = timestamp

        omega_vec: _FLOAT_ARRAY = _as_vector(omega, "omega")
        omega_cov_mat: _FLOAT_ARRAY = _as_matrix(omega_cov, "omega_cov")

        omega_norm: float = float(np.linalg.norm(omega_vec))
        omega_c: _FLOAT_ARRAY = omega_vec - self._gyro_bias
        omega_c_norm: float = float(np.linalg.norm(omega_c))

        d2, inversion_ok, used_fallback = self._compute_mahalanobis(
            omega_c,
            omega_cov_mat,
            omega_c_norm,
        )

        duty_zero: bool = _is_duty_zero(
            self._state.duty_cycle,
            self._config.duty_requires_exact_zero,
            self._config.duty_zero_epsilon,
        )

        if self._config.use_adaptive_gates and duty_zero:
            self._adaptive_d2_ewma = _update_ewma(
                self._adaptive_d2_ewma,
                d2,
                self._config.adaptive_ewma_alpha,
            )
            scale: float = _clamp(
                (self._adaptive_d2_ewma or d2) / self._config.gate_enter_chi2,
                self._config.adaptive_gate_scale_min,
                self._config.adaptive_gate_scale_max,
            )
            self._state.adaptive_gate_scale = scale
        elif not self._config.use_adaptive_gates:
            self._state.adaptive_gate_scale = 1.0

        gate_enter: float = (
            self._config.gate_enter_chi2 * self._state.adaptive_gate_scale
        )
        gate_exit: float = self._config.gate_exit_chi2 * self._state.adaptive_gate_scale

        if used_fallback:
            quiet: bool = duty_zero and (omega_c_norm < self._config.omega_norm_enter)
            loud: bool = (not duty_zero) or (
                omega_c_norm > self._config.omega_norm_exit
            )
            reason: str = "norm_gate"
        else:
            quiet = duty_zero and (d2 < gate_enter)
            loud = (not duty_zero) or (d2 > gate_exit)
            reason = "mahalanobis_gate" if inversion_ok else "regularized_gate"

        stationary: bool = self._state.stationary
        candidate_quiet_time: float = self._state.candidate_quiet_time
        candidate_loud_time: float = self._state.candidate_loud_time
        stationary_dwell: float = self._state.stationary_dwell_time

        if stationary:
            if loud:
                candidate_loud_time += dt
            else:
                candidate_loud_time = 0.0
            if candidate_loud_time >= self._config.min_exit_sec:
                stationary = False
                stationary_dwell = 0.0
                candidate_loud_time = 0.0
                candidate_quiet_time = 0.0
                reason = "exit_stationary"
        else:
            if quiet:
                candidate_quiet_time += dt
            else:
                candidate_quiet_time = 0.0
            if candidate_quiet_time >= self._config.min_stationary_sec:
                stationary = True
                stationary_dwell = 0.0
                candidate_quiet_time = 0.0
                candidate_loud_time = 0.0
                reason = "enter_stationary"

        if stationary:
            stationary_dwell += dt
        else:
            stationary_dwell = 0.0

        self._state.stationary = stationary
        self._state.candidate_quiet_time = candidate_quiet_time
        self._state.candidate_loud_time = candidate_loud_time
        self._state.stationary_dwell_time = stationary_dwell
        self._state.last_d2 = d2
        self._state.last_omega_norm = omega_norm
        self._state.last_omega_c_norm = omega_c_norm
        self._state.last_gate_enter = gate_enter
        self._state.last_gate_exit = gate_exit
        self._state.last_reason = reason

        zupt_vx_variance: Optional[float]
        if stationary:
            zupt_vx_variance = _anneal_variance(
                stationary_dwell,
                self._config.zupt_sigma_start_mps,
                self._config.zupt_sigma_min_mps,
                self._config.zupt_anneal_tau_sec,
            )
        else:
            zupt_vx_variance = None

        diagnostics: dict[str, object] = {
            "dt_sec": dt,
            "dt_clamped": dt_clamped,
            "duty_zero": duty_zero,
            "omega_norm": omega_norm,
            "omega_c_norm": omega_c_norm,
            "d2": d2,
            "gate_enter": gate_enter,
            "gate_exit": gate_exit,
            "adaptive_gate_scale": self._state.adaptive_gate_scale,
            "used_fallback": used_fallback,
            "reason": reason,
        }

        return {
            "stationary": stationary,
            "should_publish_zupt": stationary,
            "stationary_dwell_sec": stationary_dwell,
            "zupt_vx_variance": zupt_vx_variance,
            "diagnostics": diagnostics,
        }

    def _compute_mahalanobis(
        self,
        omega_c: _FLOAT_ARRAY,
        omega_cov: _FLOAT_ARRAY,
        omega_c_norm: float,
    ) -> tuple[float, bool, bool]:
        """Compute Mahalanobis distance squared with regularization.

        Returns:
            Tuple of (d2, inversion_ok, used_fallback)
        """

        if not np.isfinite(omega_c_norm):
            return float("inf"), False, True

        cov_sum: _FLOAT_ARRAY = omega_cov + self._gyro_bias_cov
        cov_sum = 0.5 * (cov_sum + cov_sum.T)
        diag: _FLOAT_ARRAY = np.diag(cov_sum)
        diag = np.maximum(diag, 0.0)
        cov_sum = cov_sum.copy()
        cov_sum[np.diag_indices_from(cov_sum)] = diag

        eps: float = self._config.covariance_regularization_eps

        # (rad/s)^2 regularization on the diagonal to ensure invertibility
        cov_reg: _FLOAT_ARRAY = cov_sum + (eps * np.eye(3, dtype=np.float64))

        try:
            solve = np.linalg.solve(cov_reg, omega_c)
        except np.linalg.LinAlgError:
            return float("inf"), False, True

        d2: float = float(omega_c.T @ solve)
        if not np.isfinite(d2):
            return float("inf"), False, True

        return d2, True, False


def _as_vector(vec: _FLOAT_ARRAY, name: str) -> _FLOAT_ARRAY:
    """Convert input to a finite 3-vector."""

    array: _FLOAT_ARRAY = np.asarray(vec, dtype=np.float64).reshape(-1)
    if array.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return cast(_FLOAT_ARRAY, array)


def _as_matrix(mat: _FLOAT_ARRAY, name: str) -> _FLOAT_ARRAY:
    """Convert input to a finite 3x3 matrix."""

    array: _FLOAT_ARRAY = np.asarray(mat, dtype=np.float64)
    if array.shape != (3, 3):
        raise ValueError(f"{name} must have shape (3, 3), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return cast(_FLOAT_ARRAY, array)


def _is_duty_zero(duty: float, exact: bool, epsilon: float) -> bool:
    """Return True when the duty cycle should be treated as zero."""

    if exact and epsilon <= 0.0:
        return duty == 0.0
    return abs(duty) <= epsilon


def _update_ewma(
    prev: Optional[float],
    value: float,
    alpha: float,
) -> float:
    """Update an EWMA value."""

    if prev is None:
        return value
    return (alpha * value) + ((1.0 - alpha) * prev)


def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between bounds."""

    return max(min_value, min(value, max_value))


def _anneal_variance(
    dwell_sec: float,
    sigma_start_mps: float,
    sigma_min_mps: float,
    tau_sec: float,
) -> float:
    """Compute annealed ZUPT measurement variance."""

    if tau_sec <= 0.0:
        return sigma_min_mps * sigma_min_mps

    # (m/s)^2 initial ZUPT variance based on start sigma
    sigma2_start: float = sigma_start_mps * sigma_start_mps

    # (m/s)^2 minimum ZUPT variance based on minimum sigma
    sigma2_min: float = sigma_min_mps * sigma_min_mps

    return sigma2_min + (sigma2_start - sigma2_min) * math.exp(-dwell_sec / tau_sec)
