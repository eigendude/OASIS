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

from oasis_control.localization.mounting.mounting_types.result_snapshot import SE3
from oasis_control.localization.mounting.mounting_types.result_snapshot import (
    ResultSnapshot,
)


def _make_se3() -> SE3:
    """Create a valid SE3 pose for tests."""
    return SE3(R=np.eye(3, dtype=np.float64), t_m=np.zeros(3, dtype=np.float64))


def test_result_snapshot_validation() -> None:
    """Ensure snapshot validates required shapes."""
    T_BI: SE3 = _make_se3()
    with pytest.raises(ValueError):
        ResultSnapshot(
            t_meas_ns=0,
            frame_base="base",
            frame_imu="imu",
            frame_mag=None,
            T_BI=T_BI,
            T_BM=None,
            cov_rot_BI=np.eye(2, dtype=np.float64),
            cov_trans_BI=np.eye(3, dtype=np.float64),
            cov_rot_BM=None,
            cov_trans_BM=None,
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
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
    T_BI: SE3 = _make_se3()
    T_BM: SE3 = _make_se3()
    snapshot: ResultSnapshot = ResultSnapshot(
        t_meas_ns=0,
        frame_base="base",
        frame_imu="imu",
        frame_mag="mag",
        T_BI=T_BI,
        T_BM=T_BM,
        cov_rot_BI=np.eye(3, dtype=np.float64),
        cov_trans_BI=np.eye(3, dtype=np.float64),
        cov_rot_BM=np.eye(3, dtype=np.float64),
        cov_trans_BM=np.eye(3, dtype=np.float64),
        b_a_mps2=np.zeros(3, dtype=np.float64),
        A_a=np.eye(3, dtype=np.float64),
        b_g_rads=np.zeros(3, dtype=np.float64),
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
            T_BI=T_BI,
            T_BM=T_BM,
            cov_rot_BI=np.eye(3, dtype=np.float64),
            cov_trans_BI=np.eye(3, dtype=np.float64),
            cov_rot_BM=np.eye(3, dtype=np.float64),
            cov_trans_BM=np.eye(3, dtype=np.float64),
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
            b_m_T=None,
            R_m=None,
            segment_count=2,
            keyframe_count=1,
            diversity_tilt_deg=None,
            diversity_yaw_deg=None,
            is_stable=False,
            quality_score=None,
        )
