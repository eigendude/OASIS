################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Adaptive noise modeling for magnetometer direction residuals."""

from __future__ import annotations

import numpy as np


class NoiseAdaptationError(Exception):
    """Raised when noise adaptation inputs are invalid."""


def _as_float_array(value: np.ndarray, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Return a float64 numpy array with a required shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise NoiseAdaptationError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise NoiseAdaptationError(f"{name} must contain finite values")
    return array


def _sorted_diagonal_bounds(matrix: np.ndarray) -> np.ndarray:
    """Return sorted diagonal bounds from a matrix."""
    diag: np.ndarray = np.diag(matrix).astype(np.float64)
    if diag.shape != (3,):
        raise NoiseAdaptationError("matrix diagonal must be length 3")
    if not np.all(np.isfinite(diag)):
        raise NoiseAdaptationError("matrix diagonal must be finite")
    return np.sort(diag)


class DirectionNoiseAdapter:
    """Adaptive covariance-matching update for direction residual noise."""

    def __init__(
        self,
        *,
        R_init: np.ndarray,
        R_min: np.ndarray,
        R_max: np.ndarray,
        alpha: float,
    ) -> None:
        """Initialize the direction-noise adapter."""
        self._R: np.ndarray = _as_float_array(R_init, "R_init", (3, 3))
        self._R_min: np.ndarray = _as_float_array(R_min, "R_min", (3, 3))
        self._R_max: np.ndarray = _as_float_array(R_max, "R_max", (3, 3))
        if not np.isfinite(alpha) or alpha <= 0.0 or alpha > 1.0:
            raise NoiseAdaptationError("alpha must be in (0, 1]")
        self._alpha: float = float(alpha)
        self._lam_min: np.ndarray = _sorted_diagonal_bounds(self._R_min)
        self._lam_max: np.ndarray = _sorted_diagonal_bounds(self._R_max)

    def R(self) -> np.ndarray:
        """Return the current noise covariance."""
        return np.array(self._R, dtype=np.float64)

    def update(self, *, r: np.ndarray, S_hat: np.ndarray) -> np.ndarray:
        """Update the noise covariance using covariance matching."""
        residual: np.ndarray = _as_float_array(r, "r", (3,))
        S_hat_mat: np.ndarray = _as_float_array(S_hat, "S_hat", (3, 3))

        outer: np.ndarray = np.outer(residual, residual)
        R_new: np.ndarray = (1.0 - self._alpha) * self._R + self._alpha * (
            outer - S_hat_mat
        )
        R_new = 0.5 * (R_new + R_new.T)

        eigvals: np.ndarray
        eigvecs: np.ndarray
        eigvals, eigvecs = np.linalg.eigh(R_new)
        clamped: np.ndarray = np.clip(eigvals, self._lam_min, self._lam_max)
        R_clamped: np.ndarray = eigvecs @ np.diag(clamped) @ eigvecs.T
        self._R = 0.5 * (R_clamped + R_clamped.T)
        return np.array(self._R, dtype=np.float64)
