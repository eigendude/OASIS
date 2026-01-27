################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for AHRS mounting node helpers."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.diagnostics.imu_pair_tracker import (
    ImuPairTracker,
)
from oasis_control.localization.mounting.math_utils.validation import (
    CovarianceValidationError,
)
from oasis_control.localization.mounting.math_utils.validation import (
    normalize_quaternion_wxyz,
)
from oasis_control.localization.mounting.math_utils.validation import reshape_covariance
from oasis_control.localization.mounting.math_utils.validation import reshape_matrix


def test_reshape_matrix_accepts_negative_diagonal() -> None:
    """Ensure accel calibration matrices allow negative diagonals."""
    values: list[float] = [
        -1.0,
        0.1,
        0.2,
        0.0,
        -0.5,
        0.3,
        0.0,
        0.0,
        -0.2,
    ]
    matrix: np.ndarray = reshape_matrix(values, (3, 3), "accel_a")
    assert matrix.shape == (3, 3)
    assert matrix[0, 0] < 0.0


def test_reshape_covariance_accepts_unknown_all_minus_one() -> None:
    """Ensure all -1 covariances are substituted."""
    values: list[float] = [-1.0] * 9
    fallback: np.ndarray = np.diag([0.1, 0.2, 0.3])
    matrix: np.ndarray = reshape_covariance(
        values,
        (3, 3),
        "covariance",
        fallback=fallback,
    )
    assert np.allclose(matrix, fallback)


def test_reshape_covariance_accepts_unknown_diag_minus_one() -> None:
    """Ensure -1 diagonal covariances are substituted."""
    values: list[float] = [
        -1.0,
        0.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        -1.0,
    ]
    fallback: np.ndarray = np.diag([0.4, 0.5, 0.6])
    matrix: np.ndarray = reshape_covariance(
        values,
        (3, 3),
        "covariance",
        fallback=fallback,
    )
    assert np.allclose(matrix, fallback)


def test_reshape_covariance_rejects_unknown_without_fallback() -> None:
    """Ensure unknown covariances still raise without fallback."""
    values: list[float] = [-1.0] * 9
    with pytest.raises(CovarianceValidationError):
        reshape_covariance(values, (3, 3), "covariance")


def test_reshape_covariance_rejects_negative_diagonal() -> None:
    """Ensure negative diagonal entries are rejected."""
    values: list[float] = [
        -0.1,
        0.0,
        0.0,
        0.0,
        0.2,
        0.0,
        0.0,
        0.0,
        0.3,
    ]
    with pytest.raises(ValueError):
        reshape_covariance(values, (3, 3), "covariance")


def test_reshape_covariance_rejects_nan_values() -> None:
    """Ensure NaN covariances are rejected."""
    values: list[float] = [
        0.1,
        0.0,
        0.0,
        0.0,
        float("nan"),
        0.0,
        0.0,
        0.0,
        0.3,
    ]
    with pytest.raises(ValueError):
        reshape_covariance(values, (3, 3), "covariance")


def test_imu_pair_tracker_rejects_bad_cov_without_unpaired_drop() -> None:
    """Ensure covariance rejection increments bad cov, not unpaired drops."""
    tracker: ImuPairTracker = ImuPairTracker(warmup_sec=0.0)
    t_ns: int = 1_000_000
    tracker.record_raw(t_ns)
    tracker.record_calibration(t_ns)
    tracker.record_pair(t_ns)
    tracker.reject_bad_cov()
    assert tracker.imu_pairs_rejected_bad_cov == 1
    assert tracker.unpaired_raw(t_ns) == 0


def test_imu_pair_tracker_counts_unpaired_raw() -> None:
    """Ensure unpaired raw messages are counted after warmup."""
    tracker: ImuPairTracker = ImuPairTracker(warmup_sec=0.0)
    t_ns: int = 2_000_000
    tracker.record_raw(t_ns)
    assert tracker.unpaired_raw(t_ns) == 1


def test_quaternion_normalization() -> None:
    """Ensure quaternion inputs are normalized."""
    wxyz: list[float] = [2.0, 0.0, 0.0, 0.0]
    normalized: np.ndarray = normalize_quaternion_wxyz(wxyz)
    assert np.isclose(normalized[0], 1.0)
    assert np.isclose(normalized[1], 0.0)
    assert np.isclose(normalized[2], 0.0)
    assert np.isclose(normalized[3], 0.0)
