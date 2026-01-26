################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for mounting YAML formatting."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.storage.yaml_format import DiversityYaml
from oasis_control.localization.mounting.storage.yaml_format import FlagsYaml
from oasis_control.localization.mounting.storage.yaml_format import FramesYaml
from oasis_control.localization.mounting.storage.yaml_format import ImuNuisanceYaml
from oasis_control.localization.mounting.storage.yaml_format import MagNuisanceYaml
from oasis_control.localization.mounting.storage.yaml_format import MountingSnapshotYaml
from oasis_control.localization.mounting.storage.yaml_format import MountingYamlError
from oasis_control.localization.mounting.storage.yaml_format import QualityYaml
from oasis_control.localization.mounting.storage.yaml_format import TransformYaml
from oasis_control.localization.mounting.storage.yaml_format import dumps_yaml
from oasis_control.localization.mounting.storage.yaml_format import loads_yaml
from oasis_control.localization.mounting.storage.yaml_format import snapshot_to_dict


def _build_snapshot() -> MountingSnapshotYaml:
    """Create a nontrivial snapshot for tests."""
    frames: FramesYaml = FramesYaml(
        base_frame="base_link",
        imu_frame="imu_link",
        mag_frame="mag_link",
    )
    flags: FlagsYaml = FlagsYaml(
        anchored=True,
        mag_reference_invalid=False,
        mag_disturbance_detected=True,
        mag_dir_prior_from_driver_cov=False,
    )
    t_bi: TransformYaml = TransformYaml(
        translation_m=np.array([0.1, -0.2, 0.3], dtype=np.float64),
        quaternion_wxyz=np.array([0.707, 0.0, 0.707, 0.0], dtype=np.float64),
        rot_cov_rad2=np.eye(3, dtype=np.float64) * 0.01,
    )
    t_bm: TransformYaml = TransformYaml(
        translation_m=np.array([0.4, 0.5, -0.6], dtype=np.float64),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        rot_cov_rad2=np.eye(3, dtype=np.float64) * 0.02,
    )
    imu: ImuNuisanceYaml = ImuNuisanceYaml(
        accel_bias_mps2=np.array([0.01, -0.02, 0.03], dtype=np.float64),
        accel_A_row_major=np.arange(9, dtype=np.float64) * 0.01,
        accel_param_cov_row_major_12x12=np.arange(144, dtype=np.float64) * 0.001,
        gyro_bias_rads=np.array([0.001, 0.002, -0.003], dtype=np.float64),
        gyro_bias_cov_row_major_3x3=np.arange(9, dtype=np.float64) * 0.002,
    )
    mag: MagNuisanceYaml = MagNuisanceYaml(
        offset_t=np.array([1e-6, -2e-6, 3e-6], dtype=np.float64),
        offset_cov_row_major_3x3=np.arange(9, dtype=np.float64) * 1e-8,
        R_m_unitless2_row_major_3x3=np.arange(9, dtype=np.float64) * 0.003,
        R_m0_unitless2_row_major_3x3=np.arange(9, dtype=np.float64) * 0.004,
    )
    diversity: DiversityYaml = DiversityYaml(
        gravity_max_angle_deg=35.0,
        mag_proj_max_angle_deg=20.0,
    )
    quality: QualityYaml = QualityYaml(
        raw_samples=120,
        steady_segments=8,
        keyframes=4,
        total_duration_sec=42.5,
        accel_residual_rms=0.12,
        mag_residual_rms=0.08,
        diversity=diversity,
    )
    return MountingSnapshotYaml(
        format_version=1,
        frames=frames,
        flags=flags,
        T_BI=t_bi,
        T_BM=t_bm,
        imu=imu,
        mag=mag,
        quality=quality,
    )


def _assert_snapshots_equal(
    left: MountingSnapshotYaml, right: MountingSnapshotYaml
) -> None:
    """Assert two snapshots are equivalent."""
    assert left.format_version == right.format_version
    assert left.frames == right.frames
    assert left.flags == right.flags
    np.testing.assert_allclose(left.T_BI.translation_m, right.T_BI.translation_m)
    np.testing.assert_allclose(left.T_BI.quaternion_wxyz, right.T_BI.quaternion_wxyz)
    np.testing.assert_allclose(left.T_BI.rot_cov_rad2, right.T_BI.rot_cov_rad2)
    np.testing.assert_allclose(left.T_BM.translation_m, right.T_BM.translation_m)
    np.testing.assert_allclose(left.T_BM.quaternion_wxyz, right.T_BM.quaternion_wxyz)
    np.testing.assert_allclose(left.T_BM.rot_cov_rad2, right.T_BM.rot_cov_rad2)
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


def test_round_trip_yaml() -> None:
    """Ensure YAML dumps/loads round-trips preserve data."""
    snapshot: MountingSnapshotYaml = _build_snapshot()

    text: str = dumps_yaml(snapshot)
    loaded: MountingSnapshotYaml = loads_yaml(text)

    _assert_snapshots_equal(snapshot, loaded)


def test_schema_missing_key() -> None:
    """Ensure missing keys raise errors."""
    snapshot: MountingSnapshotYaml = _build_snapshot()
    data: dict[str, object] = snapshot_to_dict(snapshot)
    data.pop("frames")

    from oasis_control.localization.mounting.storage.yaml_format import (
        snapshot_from_dict,
    )

    with pytest.raises(MountingYamlError):
        snapshot_from_dict(data)


def test_schema_wrong_shapes() -> None:
    """Ensure wrong shapes raise errors."""
    with pytest.raises(MountingYamlError):
        TransformYaml(
            translation_m=np.array([0.0, 0.0], dtype=np.float64),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            rot_cov_rad2=np.eye(3, dtype=np.float64),
        )

    with pytest.raises(MountingYamlError):
        TransformYaml(
            translation_m=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0], dtype=np.float64),
            rot_cov_rad2=np.eye(3, dtype=np.float64),
        )

    with pytest.raises(MountingYamlError):
        TransformYaml(
            translation_m=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            rot_cov_rad2=np.eye(2, dtype=np.float64),
        )


def test_schema_format_version() -> None:
    """Ensure format_version must be 1."""
    snapshot: MountingSnapshotYaml = _build_snapshot()
    data: dict[str, object] = snapshot_to_dict(snapshot)
    data["format_version"] = 2

    from oasis_control.localization.mounting.storage.yaml_format import (
        snapshot_from_dict,
    )

    with pytest.raises(MountingYamlError):
        snapshot_from_dict(data)


def test_schema_unknown_top_level_key() -> None:
    """Ensure unknown top-level keys are rejected."""
    snapshot: MountingSnapshotYaml = _build_snapshot()
    data: dict[str, object] = snapshot_to_dict(snapshot)
    data["unexpected"] = "value"

    from oasis_control.localization.mounting.storage.yaml_format import (
        snapshot_from_dict,
    )

    with pytest.raises(MountingYamlError):
        snapshot_from_dict(data)


def test_dump_deterministic() -> None:
    """Ensure YAML dumps are deterministic."""
    snapshot: MountingSnapshotYaml = _build_snapshot()

    first: str = dumps_yaml(snapshot)
    second: str = dumps_yaml(snapshot)

    assert first == second
