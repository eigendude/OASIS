################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for keyframe clustering."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from oasis_control.localization.mounting.config.mounting_params import ClusterParams
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.models.keyframe_clustering import (
    KeyframeClustering,
)
from oasis_control.localization.mounting.models.keyframe_clustering import (
    KeyframeClusteringError,
)
from oasis_control.localization.mounting.mounting_types import SteadySegment


def _segment_from_dirs(
    *,
    t_start_ns: int,
    gravity_dir_I: np.ndarray,
    mag_dir_M: np.ndarray | None = None,
) -> SteadySegment:
    """Create a steady segment from direction inputs."""
    gravity_unit: np.ndarray = gravity_dir_I / float(np.linalg.norm(gravity_dir_I))
    a_mean: np.ndarray = -gravity_unit * 9.81
    cov: np.ndarray = np.eye(3, dtype=np.float64) * 1e-3
    if mag_dir_M is None:
        m_mean: np.ndarray | None = None
        cov_m: np.ndarray | None = None
        mag_frame_id: str | None = None
    else:
        m_mean = mag_dir_M / float(np.linalg.norm(mag_dir_M))
        cov_m = np.eye(3, dtype=np.float64) * 1e-4
        mag_frame_id = "mag"
    t_end_ns: int = t_start_ns + int(1e9)
    return SteadySegment(
        t_start_ns=t_start_ns,
        t_end_ns=t_end_ns,
        t_meas_ns=(t_start_ns + t_end_ns) // 2,
        imu_frame_id="imu",
        mag_frame_id=mag_frame_id,
        a_mean_mps2=a_mean,
        cov_a=cov,
        m_mean_T=m_mean,
        cov_m=cov_m,
        sample_count=10,
        duration_ns=t_end_ns - t_start_ns,
    )


def _params_with_cluster(cluster: ClusterParams) -> MountingParams:
    """Return mounting params with custom cluster parameters."""
    base_params: MountingParams = MountingParams.defaults()
    return replace(base_params, cluster=cluster)


def test_cluster_assigns_nearby_segments() -> None:
    """Ensure nearby segments are assigned to the same keyframe."""
    cluster_params: ClusterParams = ClusterParams(
        cluster_g_deg=15.0, cluster_h_deg=20.0
    )
    params: MountingParams = _params_with_cluster(cluster_params)
    clustering: KeyframeClustering = KeyframeClustering(params)

    seg0: SteadySegment = _segment_from_dirs(
        t_start_ns=0,
        gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
    )
    key_id0, keyframe0 = clustering.add_segment(seg0)
    assert key_id0 == 0

    seg1: SteadySegment = _segment_from_dirs(
        t_start_ns=int(2e9),
        gravity_dir_I=np.array([0.0, 0.1, 0.995], dtype=np.float64),
    )
    key_id1, keyframe1 = clustering.add_segment(seg1)
    assert key_id1 == key_id0
    assert keyframe1.segment_count == keyframe0.segment_count + 1

    seg2: SteadySegment = _segment_from_dirs(
        t_start_ns=int(4e9),
        gravity_dir_I=np.array([1.0, 0.0, 0.0], dtype=np.float64),
    )
    key_id2, _ = clustering.add_segment(seg2)
    assert key_id2 != key_id0


def test_mag_threshold_affects_clustering() -> None:
    """Ensure mag angular threshold influences keyframe assignment."""
    cluster_params: ClusterParams = ClusterParams(cluster_g_deg=20.0, cluster_h_deg=5.0)
    params: MountingParams = _params_with_cluster(cluster_params)
    clustering: KeyframeClustering = KeyframeClustering(params)

    seg0: SteadySegment = _segment_from_dirs(
        t_start_ns=0,
        gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        mag_dir_M=np.array([1.0, 0.0, 0.0], dtype=np.float64),
    )
    clustering.add_segment(seg0)

    seg1: SteadySegment = _segment_from_dirs(
        t_start_ns=int(2e9),
        gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        mag_dir_M=np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    )
    key_id1, _ = clustering.add_segment(seg1)
    assert key_id1 == 1


def test_kmax_oldest_policy() -> None:
    """Ensure the oldest keyframe is dropped when exceeding K_max."""
    cluster_params: ClusterParams = ClusterParams(
        cluster_g_deg=5.0,
        cluster_h_deg=5.0,
        K_max=2,
        drop_policy="oldest",
    )
    params: MountingParams = _params_with_cluster(cluster_params)
    clustering: KeyframeClustering = KeyframeClustering(params)

    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=0,
            gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        )
    )
    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=int(2e9),
            gravity_dir_I=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        )
    )
    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=int(4e9),
            gravity_dir_I=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        )
    )

    keyframe_ids: list[int] = [key.keyframe_id for key in clustering.keyframes()]
    assert keyframe_ids == [1, 2]


def test_kmax_lowest_information_policy() -> None:
    """Ensure lowest information keyframe is dropped when full."""
    cluster_params: ClusterParams = ClusterParams(
        cluster_g_deg=5.0,
        cluster_h_deg=5.0,
        K_max=2,
        drop_policy="lowest_information",
    )
    params: MountingParams = _params_with_cluster(cluster_params)
    clustering: KeyframeClustering = KeyframeClustering(params)

    seg0: SteadySegment = _segment_from_dirs(
        t_start_ns=0,
        gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
    )
    clustering.add_segment(seg0)
    clustering.add_segment(seg0)

    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=int(2e9),
            gravity_dir_I=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        )
    )
    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=int(4e9),
            gravity_dir_I=np.array([0.0, 1.0, 0.0], dtype=np.float64),
        )
    )

    keyframe_ids: list[int] = [key.keyframe_id for key in clustering.keyframes()]
    assert keyframe_ids == [0, 2]


def test_kmax_redundant_policy() -> None:
    """Ensure redundant keyframes are dropped deterministically."""
    cluster_params: ClusterParams = ClusterParams(
        cluster_g_deg=1.0,
        cluster_h_deg=5.0,
        K_max=2,
        drop_policy="redundant",
    )
    params: MountingParams = _params_with_cluster(cluster_params)
    clustering: KeyframeClustering = KeyframeClustering(params)

    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=0,
            gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        )
    )
    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=int(2e9),
            gravity_dir_I=np.array([0.0, 0.035, 0.9994], dtype=np.float64),
        )
    )
    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=int(4e9),
            gravity_dir_I=np.array([1.0, 0.0, 0.0], dtype=np.float64),
        )
    )

    keyframe_ids: list[int] = [key.keyframe_id for key in clustering.keyframes()]
    assert keyframe_ids == [1, 2]


def test_unknown_drop_policy_raises() -> None:
    """Ensure invalid drop policies raise errors."""
    cluster_params: ClusterParams = ClusterParams(
        cluster_g_deg=5.0,
        cluster_h_deg=5.0,
        K_max=1,
        drop_policy="unknown",
    )
    params: MountingParams = _params_with_cluster(cluster_params)
    clustering: KeyframeClustering = KeyframeClustering(params)

    clustering.add_segment(
        _segment_from_dirs(
            t_start_ns=0,
            gravity_dir_I=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        )
    )
    with pytest.raises(KeyframeClusteringError):
        clustering.add_segment(
            _segment_from_dirs(
                t_start_ns=int(2e9),
                gravity_dir_I=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            )
        )
