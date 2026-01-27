################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for keyframe aggregation types."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.mounting_types.keyframe import Keyframe
from oasis_control.localization.mounting.mounting_types.steady_segment import (
    SteadySegment,
)


def _empty_keyframe() -> Keyframe:
    """Create an empty keyframe with zero counts."""
    return Keyframe(
        keyframe_id=0,
        gravity_mean_dir_I=np.zeros(3, dtype=np.float64),
        gravity_cov_dir_I=np.zeros((3, 3), dtype=np.float64),
        gravity_weight=0,
        omega_mean_rads_raw=np.zeros(3, dtype=np.float64),
        cov_omega_raw=np.zeros((3, 3), dtype=np.float64),
        omega_weight=0,
        accel_mean_mps2_raw=np.zeros(3, dtype=np.float64),
        cov_accel_raw=np.zeros((3, 3), dtype=np.float64),
        accel_weight=0,
        mag_mean_dir_M=None,
        mag_cov_dir_M=None,
        mag_weight=0,
        segment_count=0,
    )


def test_keyframe_update_with_directions() -> None:
    """Ensure updates adjust counts and mean directions."""
    keyframe: Keyframe = _empty_keyframe()
    keyframe = keyframe.update_with_directions(np.array([1.0, 0.0, 0.0]))
    assert keyframe.gravity_weight == 1
    assert keyframe.segment_count == 1
    np.testing.assert_allclose(keyframe.gravity_mean_dir_I, np.array([1.0, 0.0, 0.0]))

    keyframe = keyframe.update_with_directions(np.array([0.0, 1.0, 0.0]))
    assert keyframe.gravity_weight == 2
    assert keyframe.segment_count == 2
    np.testing.assert_allclose(
        keyframe.gravity_mean_dir_I,
        np.array([0.5, 0.5, 0.0]),
    )
    unit_mean: np.ndarray = keyframe.gravity_unit_mean_dir_I()
    np.testing.assert_allclose(
        unit_mean,
        np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0),
    )


def test_keyframe_update_with_segment() -> None:
    """Ensure segments update gravity and mag aggregates."""
    keyframe: Keyframe = _empty_keyframe()
    segment: SteadySegment = SteadySegment(
        t_start_ns=0,
        t_end_ns=10,
        t_meas_ns=5,
        imu_frame_id="imu",
        mag_frame_id="mag",
        a_mean_mps2=np.array([0.0, 0.0, 9.81], dtype=np.float64),
        cov_a=np.eye(3, dtype=np.float64),
        omega_mean_rads_raw=np.zeros(3, dtype=np.float64),
        cov_omega_raw=np.eye(3, dtype=np.float64),
        accel_mean_mps2_raw=np.array([0.0, 0.0, 9.81], dtype=np.float64),
        cov_accel_raw=np.eye(3, dtype=np.float64),
        m_mean_T=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        cov_m=np.eye(3, dtype=np.float64),
        sample_count=2,
        duration_ns=10,
    )
    keyframe = keyframe.update_with_segment(segment)
    assert keyframe.gravity_weight == 1
    assert keyframe.mag_weight == 1
    assert keyframe.segment_count == 1
    assert keyframe.mag_mean_dir_M is not None


def test_keyframe_requires_non_zero_mean_when_weighted() -> None:
    """Ensure non-zero mean vectors are required with positive weight."""
    with pytest.raises(ValueError):
        Keyframe(
            keyframe_id=1,
            gravity_mean_dir_I=np.zeros(3, dtype=np.float64),
            gravity_cov_dir_I=np.zeros((3, 3), dtype=np.float64),
            gravity_weight=1,
            omega_mean_rads_raw=np.zeros(3, dtype=np.float64),
            cov_omega_raw=np.zeros((3, 3), dtype=np.float64),
            omega_weight=0,
            accel_mean_mps2_raw=np.zeros(3, dtype=np.float64),
            cov_accel_raw=np.zeros((3, 3), dtype=np.float64),
            accel_weight=0,
            mag_mean_dir_M=None,
            mag_cov_dir_M=None,
            mag_weight=0,
            segment_count=0,
        )
