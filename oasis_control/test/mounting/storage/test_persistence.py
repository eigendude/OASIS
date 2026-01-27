################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for mounting YAML persistence."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from oasis_control.localization.mounting.storage.persistence import (
    MountingPersistenceError,
)
from oasis_control.localization.mounting.storage.persistence import load_yaml_snapshot
from oasis_control.localization.mounting.storage.persistence import save_yaml_snapshot
from oasis_control.localization.mounting.storage.yaml_format import DiversityYaml
from oasis_control.localization.mounting.storage.yaml_format import FlagsYaml
from oasis_control.localization.mounting.storage.yaml_format import FramesYaml
from oasis_control.localization.mounting.storage.yaml_format import ImuNuisanceYaml
from oasis_control.localization.mounting.storage.yaml_format import MagNuisanceYaml
from oasis_control.localization.mounting.storage.yaml_format import MountingSnapshotYaml
from oasis_control.localization.mounting.storage.yaml_format import MountRotationYaml
from oasis_control.localization.mounting.storage.yaml_format import QualityYaml


def _build_snapshot() -> MountingSnapshotYaml:
    """Create a snapshot for persistence tests."""
    frames: FramesYaml = FramesYaml(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
    )
    flags: FlagsYaml = FlagsYaml(
        anchored=False,
        mag_reference_invalid=False,
        mag_disturbance_detected=False,
        mag_dir_prior_from_driver_cov=True,
    )
    r_bi: MountRotationYaml = MountRotationYaml(
        quaternion_wxyz=np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float64),
        rot_cov_rad2=np.eye(3, dtype=np.float64) * 0.05,
    )
    r_bm: MountRotationYaml = MountRotationYaml(
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rot_cov_rad2=np.eye(3, dtype=np.float64) * 0.06,
    )
    imu: ImuNuisanceYaml = ImuNuisanceYaml(
        accel_bias_mps2=np.array([0.2, 0.1, -0.1], dtype=np.float64),
        accel_A_row_major=np.arange(9, dtype=np.float64) * 0.02,
        accel_param_cov_row_major_12x12=np.arange(144, dtype=np.float64) * 0.002,
        gyro_bias_rads=np.array([0.01, -0.02, 0.03], dtype=np.float64),
        gyro_bias_cov_row_major_3x3=np.arange(9, dtype=np.float64) * 0.003,
    )
    mag: MagNuisanceYaml = MagNuisanceYaml(
        offset_t=np.array([2e-6, 1e-6, -1e-6], dtype=np.float64),
        offset_cov_row_major_3x3=np.arange(9, dtype=np.float64) * 2e-8,
        R_m_unitless2_row_major_3x3=np.arange(9, dtype=np.float64) * 0.007,
        R_m0_unitless2_row_major_3x3=np.arange(9, dtype=np.float64) * 0.008,
    )
    diversity: DiversityYaml = DiversityYaml(
        gravity_max_angle_deg=10.0,
        mag_proj_max_angle_deg=5.0,
    )
    quality: QualityYaml = QualityYaml(
        raw_samples=10,
        steady_segments=2,
        keyframes=1,
        total_duration_sec=3.0,
        accel_residual_rms=0.2,
        mag_residual_rms=0.3,
        diversity=diversity,
    )
    return MountingSnapshotYaml(
        format_version=1,
        frames=frames,
        flags=flags,
        R_BI=r_bi,
        R_BM=r_bm,
        imu=imu,
        mag=mag,
        quality=quality,
    )


def _assert_snapshot_equivalent(
    left: MountingSnapshotYaml, right: MountingSnapshotYaml
) -> None:
    """Assert two snapshots are equivalent."""
    assert left.format_version == right.format_version
    assert left.frames == right.frames
    assert left.flags == right.flags
    np.testing.assert_allclose(left.R_BI.quaternion_wxyz, right.R_BI.quaternion_wxyz)
    np.testing.assert_allclose(left.R_BI.rot_cov_rad2, right.R_BI.rot_cov_rad2)
    np.testing.assert_allclose(left.R_BM.quaternion_wxyz, right.R_BM.quaternion_wxyz)
    np.testing.assert_allclose(left.R_BM.rot_cov_rad2, right.R_BM.rot_cov_rad2)
    np.testing.assert_allclose(left.imu.accel_bias_mps2, right.imu.accel_bias_mps2)
    np.testing.assert_allclose(left.imu.accel_A_row_major, right.imu.accel_A_row_major)
    np.testing.assert_allclose(
        left.imu.accel_param_cov_row_major_12x12,
        right.imu.accel_param_cov_row_major_12x12,
    )
    np.testing.assert_allclose(left.imu.gyro_bias_rads, right.imu.gyro_bias_rads)
    np.testing.assert_allclose(
        left.imu.gyro_bias_cov_row_major_3x3,
        right.imu.gyro_bias_cov_row_major_3x3,
    )
    np.testing.assert_allclose(left.mag.offset_t, right.mag.offset_t)
    np.testing.assert_allclose(
        left.mag.offset_cov_row_major_3x3, right.mag.offset_cov_row_major_3x3
    )
    np.testing.assert_allclose(
        left.mag.R_m_unitless2_row_major_3x3, right.mag.R_m_unitless2_row_major_3x3
    )
    np.testing.assert_allclose(
        left.mag.R_m0_unitless2_row_major_3x3, right.mag.R_m0_unitless2_row_major_3x3
    )
    assert left.quality == right.quality


def test_save_and_load_snapshot(tmp_path: Path) -> None:
    """Ensure snapshots are persisted and loaded correctly."""
    snapshot: MountingSnapshotYaml = _build_snapshot()
    path: Path = tmp_path / "mount.yaml"

    save_yaml_snapshot(path, snapshot)
    loaded: MountingSnapshotYaml = load_yaml_snapshot(path)

    _assert_snapshot_equivalent(snapshot, loaded)


def test_atomic_write(tmp_path: Path) -> None:
    """Ensure atomic writes create a valid YAML file."""
    snapshot: MountingSnapshotYaml = _build_snapshot()
    path: Path = tmp_path / "atomic.yaml"

    save_yaml_snapshot(path, snapshot, atomic_write=True)

    assert path.exists()
    loaded: MountingSnapshotYaml = load_yaml_snapshot(path)
    _assert_snapshot_equivalent(snapshot, loaded)


def test_extension_gating(tmp_path: Path) -> None:
    """Ensure only YAML extensions are accepted."""
    snapshot: MountingSnapshotYaml = _build_snapshot()
    invalid_path: Path = tmp_path / "mount.json"

    with pytest.raises(MountingPersistenceError):
        save_yaml_snapshot(invalid_path, snapshot)

    with pytest.raises(MountingPersistenceError):
        load_yaml_snapshot(invalid_path)
