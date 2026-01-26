################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for mounting rotation stability tracking."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.linalg import SO3
from oasis_control.localization.mounting.tf.stability import RotationStabilityTracker
from oasis_control.localization.mounting.tf.stability import StabilityError
from oasis_control.localization.mounting.tf.stability import StabilityStatus


def _ns(seconds: float) -> int:
    """Convert seconds to integer nanoseconds."""
    return int(seconds * 1e9)


def _identity_rotation() -> NDArray[np.float64]:
    """Return a 3x3 identity rotation matrix."""
    return np.eye(3, dtype=np.float64)


def test_stability_window_fill_requirement() -> None:
    """Require the stable window to be fully populated."""
    tracker: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.1,
    )
    R_identity: NDArray[np.float64] = _identity_rotation()

    status0: StabilityStatus = tracker.update(
        t_ns=_ns(0.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    assert not status0.is_stable

    status1: StabilityStatus = tracker.update(
        t_ns=_ns(0.5),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    assert not status1.is_stable

    status2: StabilityStatus = tracker.update(
        t_ns=_ns(1.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    assert status2.is_stable


def test_stability_delta_computation() -> None:
    """Compute maximum rotation deltas over the stable window."""
    tracker: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.05,
    )
    R_identity: NDArray[np.float64] = _identity_rotation()
    rotvec: NDArray[np.float64] = np.array([0.01, 0.0, 0.0], dtype=np.float64)
    R_small: NDArray[np.float64] = SO3.exp(rotvec)

    tracker.update(
        t_ns=_ns(0.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    status: StabilityStatus = tracker.update(
        t_ns=_ns(1.0),
        R_BI=R_small,
        R_BM=R_small,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    assert status.is_stable
    np.testing.assert_allclose(status.delta_theta_BI_rad, 0.01, atol=1e-6)
    np.testing.assert_allclose(status.delta_theta_BM_rad, 0.01, atol=1e-6)

    tracker_large: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.05,
    )
    rotvec_large: NDArray[np.float64] = np.array([0.2, 0.0, 0.0], dtype=np.float64)
    R_large: NDArray[np.float64] = SO3.exp(rotvec_large)

    tracker_large.update(
        t_ns=_ns(0.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    status_large: StabilityStatus = tracker_large.update(
        t_ns=_ns(1.0),
        R_BI=R_large,
        R_BM=R_large,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    assert not status_large.is_stable


def test_stability_gate_behavior() -> None:
    """Apply tilt and yaw gating when determining stability."""
    tracker_tilt: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.1,
    )
    R_identity: NDArray[np.float64] = _identity_rotation()

    tracker_tilt.update(
        t_ns=_ns(0.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    status_tilt: StabilityStatus = tracker_tilt.update(
        t_ns=_ns(1.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=True,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    assert not status_tilt.is_stable

    tracker_yaw: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.1,
    )
    tracker_yaw.update(
        t_ns=_ns(0.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    status_yaw: StabilityStatus = tracker_yaw.update(
        t_ns=_ns(1.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=True,
        mag_used_for_yaw=True,
    )
    assert not status_yaw.is_stable

    tracker_mag_unused: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.1,
    )
    tracker_mag_unused.update(
        t_ns=_ns(0.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=False,
    )
    status_mag_unused: StabilityStatus = tracker_mag_unused.update(
        t_ns=_ns(1.0),
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=True,
        mag_used_for_yaw=False,
    )
    assert status_mag_unused.is_stable


def test_stability_timestamp_validation() -> None:
    """Reject decreasing timestamps."""
    tracker: RotationStabilityTracker = RotationStabilityTracker(
        stable_window_sec=1.0,
        stable_rot_thresh_rad=0.1,
    )
    R_identity: NDArray[np.float64] = _identity_rotation()

    tracker.update(
        t_ns=10,
        R_BI=R_identity,
        R_BM=R_identity,
        need_more_tilt=False,
        need_more_yaw=False,
        mag_used_for_yaw=True,
    )
    with pytest.raises(StabilityError):
        tracker.update(
            t_ns=5,
            R_BI=R_identity,
            R_BM=R_identity,
            need_more_tilt=False,
            need_more_yaw=False,
            mag_used_for_yaw=True,
        )
