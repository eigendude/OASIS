################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for running statistics utilities."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.stats import RunningMoments3
from oasis_control.localization.mounting.math_utils.stats import SlidingWindowMoments3


def test_running_moments_matches_numpy() -> None:
    """Checks running moments against numpy mean/cov."""
    data: NDArray[np.float64] = np.array(
        [
            [1.0, 2.0, 3.0],
            [2.0, 0.0, 4.0],
            [3.0, 1.0, 5.0],
        ],
        dtype=float,
    )
    moments: RunningMoments3 = RunningMoments3()
    for row in data:
        row_vec: NDArray[np.float64] = row
        moments.update(row_vec)
    mean_expected: NDArray[np.float64] = np.mean(data, axis=0)
    cov_expected: NDArray[np.float64] = np.cov(data, rowvar=False, ddof=0)
    assert np.allclose(moments.mean(), mean_expected)
    assert np.allclose(moments.covariance(ddof=0), cov_expected)


def test_sliding_window_behavior() -> None:
    """Checks sliding window drops old samples."""
    window: SlidingWindowMoments3 = SlidingWindowMoments3(window_size=2)
    window.push(np.array([1.0, 0.0, 0.0], dtype=float))
    window.push(np.array([3.0, 0.0, 0.0], dtype=float))
    window.push(np.array([5.0, 0.0, 0.0], dtype=float))
    mean: NDArray[np.float64] = window.mean()
    assert np.allclose(mean, np.array([4.0, 0.0, 0.0], dtype=float))


def test_covariance_shapes_and_symmetry() -> None:
    """Checks covariance shapes and symmetry."""
    moments: RunningMoments3 = RunningMoments3()
    samples: NDArray[np.float64] = np.array(
        [[1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 0.0, 1.0]], dtype=float
    )
    for row in samples:
        row_vec: NDArray[np.float64] = row
        moments.update(row_vec)
    cov: NDArray[np.float64] = moments.covariance(ddof=0)
    assert cov.shape == (3, 3)
    assert np.allclose(cov, cov.T)


def test_invalid_shape_raises() -> None:
    """Checks invalid shapes raise errors."""
    moments: RunningMoments3 = RunningMoments3()
    with pytest.raises(ValueError):
        moments.update(np.array([1.0, 2.0], dtype=float))
    window: SlidingWindowMoments3 = SlidingWindowMoments3(window_size=3)
    with pytest.raises(ValueError):
        window.push(np.array([1.0, 2.0], dtype=float))
