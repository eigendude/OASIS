################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for mounting configuration wrapper."""

from __future__ import annotations

import dataclasses

import pytest

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_config import (
    MountingConfigError,
)
from oasis_control.localization.mounting.config.mounting_params import MountingParams


def test_defaults_construct() -> None:
    """Default parameters should construct a MountingConfig."""
    params: MountingParams = MountingParams.defaults()
    MountingConfig(params)


def test_invalid_save_format() -> None:
    """Invalid save.format should raise an error."""
    params: MountingParams = MountingParams.defaults().replace(
        save=dataclasses.replace(MountingParams.defaults().save, format="toml")
    )
    with pytest.raises(MountingConfigError):
        MountingConfig(params)


def test_invalid_drop_policy() -> None:
    """Invalid cluster.drop_policy should raise an error."""
    params: MountingParams = MountingParams.defaults().replace(
        cluster=dataclasses.replace(
            MountingParams.defaults().cluster, drop_policy="newest"
        )
    )
    with pytest.raises(MountingConfigError):
        MountingConfig(params)


def test_invalid_window_type() -> None:
    """Invalid steady.window_type should raise an error."""
    params: MountingParams = MountingParams.defaults().replace(
        steady=dataclasses.replace(
            MountingParams.defaults().steady, window_type="fixed"
        )
    )
    with pytest.raises(MountingConfigError):
        MountingConfig(params)


def test_accessors() -> None:
    """Accessor methods should return expected values."""
    params: MountingParams = MountingParams.defaults()
    config: MountingConfig = MountingConfig(params)

    assert config.base_frame() == params.frames.base_frame
    assert config.imu_raw_topic() == params.topics.imu_raw
    assert config.mag_topic() == params.topics.magnetic_field
    assert config.baseline_enabled() == params.mount.baseline_hard_enabled
    assert config.baseline_distance_m() == params.mount.baseline_im_to_mag_m


def test_baseline_defense_in_depth() -> None:
    """Baseline constraint should be enforced in MountingConfig too."""
    params: MountingParams = MountingParams.defaults().replace(
        mount=dataclasses.replace(
            MountingParams.defaults().mount,
            baseline_hard_enabled=True,
            baseline_im_to_mag_m=None,
        )
    )
    with pytest.raises(MountingConfigError):
        MountingConfig(params)
