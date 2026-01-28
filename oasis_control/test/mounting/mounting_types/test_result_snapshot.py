################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for result snapshot types."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from oasis_control.localization.mounting.mounting_types.result_snapshot import (
    ResultSnapshot,
)


def _make_rotation() -> NDArray[np.float64]:
    """Create a valid rotation matrix for tests."""
    return np.eye(3, dtype=np.float64)


def test_result_snapshot_validation() -> None:
    """Ensure snapshot validates required shapes."""
    R_BI: NDArray[np.float64] = _make_rotation()
    with pytest.raises(ValueError):
        ResultSnapshot(
            t_meas_ns=0,
            frame_base="base",
            frame_imu="imu",
            frame_mag=None,
            R_BI=R_BI,
            R_BM=None,
            cov_rot_BI=np.eye(2, dtype=np.float64),
            cov_rot_BM=None,
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            accel_param_cov=np.eye(12, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
            gyro_bias_cov=np.eye(3, dtype=np.float64),
            b_m_T=None,
            R_m=None,
            segment_count=0,
            keyframe_count=0,
            diversity_tilt_deg=None,
            diversity_yaw_deg=None,
            is_stable=False,
            quality_score=None,
        )


def test_result_snapshot_mag_consistency() -> None:
    """Ensure mag fields are consistent with mag frame."""
    R_BI: NDArray[np.float64] = _make_rotation()
    R_BM: NDArray[np.float64] = _make_rotation()
    snapshot: ResultSnapshot = ResultSnapshot(
        t_meas_ns=0,
        frame_base="base",
        frame_imu="imu",
        frame_mag="mag",
        R_BI=R_BI,
        R_BM=R_BM,
        cov_rot_BI=np.eye(3, dtype=np.float64),
        cov_rot_BM=np.eye(3, dtype=np.float64),
        b_a_mps2=np.zeros(3, dtype=np.float64),
        A_a=np.eye(3, dtype=np.float64),
        accel_param_cov=np.eye(12, dtype=np.float64),
        b_g_rads=np.zeros(3, dtype=np.float64),
        gyro_bias_cov=np.eye(3, dtype=np.float64),
        b_m_T=np.zeros(3, dtype=np.float64),
        R_m=np.eye(3, dtype=np.float64),
        segment_count=2,
        keyframe_count=1,
        diversity_tilt_deg=10.0,
        diversity_yaw_deg=20.0,
        is_stable=True,
        quality_score=0.9,
    )
    assert snapshot.frame_mag == "mag"

    with pytest.raises(ValueError):
        ResultSnapshot(
            t_meas_ns=0,
            frame_base="base",
            frame_imu="imu",
            frame_mag=None,
            R_BI=R_BI,
            R_BM=R_BM,
            cov_rot_BI=np.eye(3, dtype=np.float64),
            cov_rot_BM=np.eye(3, dtype=np.float64),
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            accel_param_cov=np.eye(12, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
            gyro_bias_cov=np.eye(3, dtype=np.float64),
            b_m_T=None,
            R_m=None,
            segment_count=2,
            keyframe_count=1,
            diversity_tilt_deg=None,
            diversity_yaw_deg=None,
            is_stable=False,
            quality_score=None,
        )
