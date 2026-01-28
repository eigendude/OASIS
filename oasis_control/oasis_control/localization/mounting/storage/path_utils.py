################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Path utilities for mounting calibration persistence."""

from __future__ import annotations

import os
from pathlib import Path


ROS_PROFILE_DIR_NAME: str = ".ros"
MOUNT_INFO_DIR_NAME: str = "mount_info"


def mount_info_directory(home: str | os.PathLike[str] | None = None) -> Path:
    """Return the mount-info directory under the ROS profile directory."""
    home_value: str | None
    if home is None:
        home_value = os.getenv("HOME")
    else:
        home_value = os.fspath(home)
    home_path: Path = Path(home_value) if home_value else Path(".")
    return home_path / ROS_PROFILE_DIR_NAME / MOUNT_INFO_DIR_NAME


def mount_calibration_path(
    *,
    base: str,
    system_id: str,
    home: str | os.PathLike[str] | None = None,
    directory: Path | None = None,
) -> Path:
    """Return the calibration YAML path for the mount."""
    if not base:
        raise ValueError("mount_calibration_base must be set")
    if not system_id:
        raise ValueError("system_id must be set")
    mount_directory: Path = (
        mount_info_directory(home) if directory is None else Path(directory)
    )
    return mount_directory / f"{base}_{system_id}.yaml"
