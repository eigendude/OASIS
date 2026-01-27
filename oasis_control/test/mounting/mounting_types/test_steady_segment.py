################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for steady segment types."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.mounting_types.steady_segment import (
    SteadySegment,
)


def test_steady_segment_gravity_direction() -> None:
    """Ensure gravity direction helper normalizes -a_mean."""
    segment: SteadySegment = SteadySegment(
        t_start_ns=0,
        t_end_ns=100,
        t_meas_ns=50,
        imu_frame_id="imu",
        mag_frame_id=None,
        a_mean_mps2=np.array([0.0, 0.0, -9.81], dtype=np.float64),
        cov_a=np.eye(3, dtype=np.float64),
        omega_mean_rads_raw=np.zeros(3, dtype=np.float64),
        cov_omega_raw=np.eye(3, dtype=np.float64),
        accel_mean_mps2_raw=np.array([0.0, 0.0, -9.81], dtype=np.float64),
        cov_accel_raw=np.eye(3, dtype=np.float64),
        m_mean_T=None,
        cov_m=None,
        sample_count=5,
        duration_ns=100,
    )
    gravity_dir: np.ndarray = segment.gravity_up_dir_I()
    np.testing.assert_allclose(gravity_dir, np.array([0.0, 0.0, 1.0]))


def test_steady_segment_mag_consistency_validation() -> None:
    """Ensure mag fields require a mag frame."""
    with pytest.raises(ValueError):
        SteadySegment(
            t_start_ns=0,
            t_end_ns=10,
            t_meas_ns=5,
            imu_frame_id="imu",
            mag_frame_id=None,
            a_mean_mps2=np.array([0.0, 0.0, 9.81], dtype=np.float64),
            cov_a=np.eye(3, dtype=np.float64),
            omega_mean_rads_raw=np.zeros(3, dtype=np.float64),
            cov_omega_raw=np.eye(3, dtype=np.float64),
            accel_mean_mps2_raw=np.array([0.0, 0.0, 9.81], dtype=np.float64),
            cov_accel_raw=np.eye(3, dtype=np.float64),
            m_mean_T=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            cov_m=np.eye(3, dtype=np.float64),
            sample_count=3,
            duration_ns=10,
        )


def test_steady_segment_finite_validation() -> None:
    """Ensure invalid values are rejected."""
    cov_a: np.ndarray = np.eye(3, dtype=np.float64)
    cov_a[0, 0] = np.nan
    with pytest.raises(ValueError):
        SteadySegment(
            t_start_ns=0,
            t_end_ns=10,
            t_meas_ns=5,
            imu_frame_id="imu",
            mag_frame_id=None,
            a_mean_mps2=np.array([0.0, 0.0, 9.81], dtype=np.float64),
            cov_a=cov_a,
            omega_mean_rads_raw=np.zeros(3, dtype=np.float64),
            cov_omega_raw=np.eye(3, dtype=np.float64),
            accel_mean_mps2_raw=np.array([0.0, 0.0, 9.81], dtype=np.float64),
            cov_accel_raw=np.eye(3, dtype=np.float64),
            m_mean_T=None,
            cov_m=None,
            sample_count=3,
            duration_ns=10,
        )
