################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import os
from pathlib import Path
from typing import Final

import yaml  # type: ignore[import-untyped]

from oasis_control.localization.speedometer.contracts import PersistenceRecord


################################################################################
# Persistence
################################################################################


DEFAULT_MOUNT_INFO_DIRECTORY: Final[Path] = Path.home() / ".ros" / "mount_info"


class ForwardYawPersistence:
    """
    YAML writer for committed forward-yaw checkpoint state.

    Startup loading is intentionally deferred. Current policy only persists the
    latest committed checkpoint for debugging and future extensions.
    """

    def __init__(self, *, hostname: str, mount_info_directory: Path) -> None:
        """Initialize the persistence helper."""

        self._hostname: str = hostname
        self._mount_info_directory: Path = mount_info_directory

    @property
    def hostname(self) -> str:
        """Return the configured host identifier."""

        return self._hostname

    @property
    def path(self) -> Path:
        """Return the convention-based persistence path."""

        return self._mount_info_directory / f"forward_twist_{self._hostname}.yaml"

    def store(self, *, record: PersistenceRecord) -> None:
        """Persist one committed forward-yaw payload as YAML."""

        self._mount_info_directory.mkdir(parents=True, exist_ok=True)
        temporary_path: Path = self.path.with_suffix(".yaml.tmp")
        with temporary_path.open("w", encoding="utf-8") as output_file:
            yaml.safe_dump(
                {
                    "version": int(record.version),
                    "created_unix_ns": int(record.created_unix_ns),
                    "host": record.hostname,
                    "estimator": record.estimator,
                    "valid": bool(record.valid),
                    "forward_yaw_rad": float(record.forward_yaw_rad),
                    "forward_axis": [
                        float(record.forward_axis_xyz[0]),
                        float(record.forward_axis_xyz[1]),
                        float(record.forward_axis_xyz[2]),
                    ],
                    "fit_sample_count": int(record.fit_sample_count),
                    "checkpoint_count": int(record.checkpoint_count),
                },
                output_file,
                sort_keys=True,
            )
        os.replace(temporary_path, self.path)
