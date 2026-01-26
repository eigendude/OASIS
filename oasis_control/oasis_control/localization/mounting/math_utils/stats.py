################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Running statistics for 3D vectors."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from dataclasses import field
from typing import Deque

import numpy as np
from numpy.typing import NDArray

from .units import assert_finite


@dataclass
class RunningMoments3:
    """Online mean and covariance for 3D vectors."""

    _count: int = 0
    _mean: NDArray[np.float64] = field(default_factory=lambda: np.zeros(3, dtype=float))
    _m2: NDArray[np.float64] = field(
        default_factory=lambda: np.zeros((3, 3), dtype=float)
    )

    def update(self, x: NDArray[np.float64]) -> None:
        """Update moments with a new sample."""
        vec: NDArray[np.float64] = np.asarray(x, dtype=float)
        if vec.shape != (3,):
            raise ValueError("x must be shape (3,)")
        assert_finite(vec, "x")
        self._count += 1
        delta: NDArray[np.float64] = vec - self._mean
        self._mean = self._mean + delta / float(self._count)
        delta2: NDArray[np.float64] = vec - self._mean
        self._m2 = self._m2 + np.outer(delta, delta2)

    def count(self) -> int:
        """Return the number of samples processed."""
        return self._count

    def mean(self) -> NDArray[np.float64]:
        """Return the current mean."""
        if self._count == 0:
            raise ValueError("No samples available")
        return np.array(self._mean, dtype=float)

    def covariance(self, ddof: int = 0) -> NDArray[np.float64]:
        """Return the covariance matrix."""
        if ddof < 0:
            raise ValueError("ddof must be non-negative")
        if self._count == 0:
            raise ValueError("No samples available")
        if self._count <= ddof:
            raise ValueError("ddof is too large for the sample count")
        scale: float = 1.0 / float(self._count - ddof)
        return self._m2 * scale

    def reset(self) -> None:
        """Reset the running statistics."""
        self._count = 0
        self._mean = np.zeros(3, dtype=float)
        self._m2 = np.zeros((3, 3), dtype=float)


class SlidingWindowMoments3:
    """Sliding-window mean and covariance for 3D vectors."""

    def __init__(self, window_size: int) -> None:
        """Create a sliding window estimator."""
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        self._window_size: int = window_size
        self._window: Deque[NDArray[np.float64]] = deque(maxlen=window_size)

    def push(self, x: NDArray[np.float64]) -> None:
        """Add a new sample to the window."""
        vec: NDArray[np.float64] = np.asarray(x, dtype=float)
        if vec.shape != (3,):
            raise ValueError("x must be shape (3,)")
        assert_finite(vec, "x")
        self._window.append(vec)

    def full(self) -> bool:
        """Return True when the window is full."""
        return len(self._window) == self._window_size

    def mean(self) -> NDArray[np.float64]:
        """Return the window mean."""
        if not self._window:
            raise ValueError("No samples available")
        data: NDArray[np.float64] = np.stack(list(self._window), axis=0)
        return np.mean(data, axis=0)

    def covariance(self, ddof: int = 0) -> NDArray[np.float64]:
        """Return the window covariance matrix."""
        if ddof < 0:
            raise ValueError("ddof must be non-negative")
        if not self._window:
            raise ValueError("No samples available")
        data: NDArray[np.float64] = np.stack(list(self._window), axis=0)
        count: int = data.shape[0]
        if count <= ddof:
            raise ValueError("ddof is too large for the sample count")
        mean: NDArray[np.float64] = np.mean(data, axis=0)
        centered: NDArray[np.float64] = data - mean
        scale: float = 1.0 / float(count - ddof)
        cov: NDArray[np.float64] = (centered.T @ centered) * scale
        return 0.5 * (cov + cov.T)
