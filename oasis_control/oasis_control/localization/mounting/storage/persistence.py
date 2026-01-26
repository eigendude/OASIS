################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Persistence helpers for mounting calibration YAML snapshots."""

from __future__ import annotations

import os
from pathlib import Path

from oasis_control.localization.mounting.storage.yaml_format import MountingSnapshotYaml
from oasis_control.localization.mounting.storage.yaml_format import MountingYamlError
from oasis_control.localization.mounting.storage.yaml_format import dumps_yaml
from oasis_control.localization.mounting.storage.yaml_format import loads_yaml


class MountingPersistenceError(Exception):
    """Raised when loading or saving mounting calibration files fails."""


def is_yaml_path(path: str | os.PathLike[str]) -> bool:
    """Return True if the path has a YAML extension."""
    suffix: str = Path(os.fspath(path)).suffix.lower()
    return suffix in {".yaml", ".yml"}


def save_yaml_snapshot(
    path: str | os.PathLike[str],
    snapshot: MountingSnapshotYaml,
    *,
    atomic_write: bool = True,
) -> None:
    """Save a mounting snapshot to disk as YAML."""
    if not is_yaml_path(path):
        raise MountingPersistenceError("Path must end with .yaml or .yml")

    path_obj: Path = Path(os.fspath(path))
    try:
        path_obj.parent.mkdir(parents=True, exist_ok=True)
        text: str = dumps_yaml(snapshot)
        if atomic_write:
            tmp_name: str = f".{path_obj.name}.tmp.{os.getpid()}"
            tmp_path: Path = path_obj.with_name(tmp_name)
            with tmp_path.open("w", encoding="utf-8") as handle:
                handle.write(text)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp_path, path_obj)
        else:
            path_obj.write_text(text, encoding="utf-8")
    except (OSError, MountingYamlError) as exc:
        raise MountingPersistenceError(
            f"Failed to save YAML snapshot to {path_obj}"
        ) from exc


def load_yaml_snapshot(path: str | os.PathLike[str]) -> MountingSnapshotYaml:
    """Load a mounting snapshot from a YAML file."""
    if not is_yaml_path(path):
        raise MountingPersistenceError("Path must end with .yaml or .yml")

    path_obj: Path = Path(os.fspath(path))
    try:
        text: str = path_obj.read_text(encoding="utf-8")
        return loads_yaml(text)
    except (OSError, MountingYamlError) as exc:
        raise MountingPersistenceError(
            f"Failed to load YAML snapshot from {path_obj}"
        ) from exc
