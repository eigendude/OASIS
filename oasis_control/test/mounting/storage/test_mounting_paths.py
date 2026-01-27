################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for mounting calibration path utilities."""

from __future__ import annotations

from pathlib import Path

import pytest

from oasis_control.localization.mounting.storage.path_utils import (
    mount_calibration_path,
)


def test_mount_calibration_path_builds() -> None:
    """Ensure the calibration path uses the expected directory and name."""
    home_dir: Path = Path("/home/oasis")
    path: Path = mount_calibration_path(
        base="ahrs_mount_calibration",
        system_id="robot01",
        home=home_dir,
    )

    assert path == (
        home_dir / ".ros" / "mount_info" / "ahrs_mount_calibration_robot01.yaml"
    )


def test_mount_calibration_path_requires_system_id() -> None:
    """Ensure a missing system_id raises a ValueError."""
    home_dir: Path = Path("/home/oasis")

    with pytest.raises(ValueError):
        mount_calibration_path(
            base="ahrs_mount_calibration",
            system_id="",
            home=home_dir,
        )
