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
    Minimal YAML writer for committed forward-yaw state.

    Startup loading is intentionally deferred. The skeleton only writes the
    committed public yaw so later work can validate file format and rollout.
    """

    def __init__(self, *, hostname: str, mount_info_directory: Path) -> None:
        """Initialize the persistence helper."""

        self._hostname: str = hostname
        self._mount_info_directory: Path = mount_info_directory

    @property
    def path(self) -> Path:
        """Return the convention-based persistence path."""

        return self._mount_info_directory / f"forward_twist_{self._hostname}.yaml"

    def store(self, *, record: PersistenceRecord) -> None:
        """Persist one committed forward-yaw payload as YAML."""

        self._mount_info_directory.mkdir(parents=True, exist_ok=True)
        with self.path.open("w", encoding="utf-8") as output_file:
            yaml.safe_dump(
                {
                    "hostname": record.hostname,
                    "forward_yaw": float(record.forward_yaw_rad),
                },
                output_file,
                sort_keys=True,
            )
