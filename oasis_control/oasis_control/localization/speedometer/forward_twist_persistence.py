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

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


PERSISTENCE_VERSION: int = 1


@dataclass(frozen=True)
class PersistedForwardYaw:
    """
    Validated persisted forward-yaw payload.

    Fields:
        forward_yaw_rad: committed yaw-only forward direction in rad
        uncertainty_forward_yaw_rad: one-sigma forward-yaw uncertainty in rad
    """

    forward_yaw_rad: float
    uncertainty_forward_yaw_rad: float


@dataclass(frozen=True)
class PersistedForwardYawMetadata:
    """
    Metadata for one committed persisted forward-yaw checkpoint.

    Fields:
        fit_sample_count: accepted learning samples used by the checkpoint
        checkpoint_count: cumulative committed checkpoint count
        confidence: dominant-axis confidence at commit
        residual: orthogonal-energy mismatch ratio at commit
        score: compact checkpoint quality score at commit
        last_update_reason: human-readable checkpoint write reason
    """

    fit_sample_count: int
    checkpoint_count: int
    confidence: float
    residual: float
    score: float
    last_update_reason: str


def load_persisted_forward_yaw(path: str) -> PersistedForwardYaw | None:
    """Load one persisted forward-yaw YAML file or return None if unusable."""

    persistence_path: Path = Path(path).expanduser()
    try:
        raw_payload: Any = yaml.safe_load(persistence_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, yaml.YAMLError):
        return None

    if not isinstance(raw_payload, dict):
        return None

    forward_yaw_rad: float = _finite_float(raw_payload.get("forward_yaw_rad"))
    uncertainty_forward_yaw_rad: float = _finite_float(
        raw_payload.get("uncertainty_forward_yaw_rad")
    )
    valid_flag: bool = bool(raw_payload.get("valid"))
    version_value: int = int(raw_payload.get("version", -1))

    if version_value != PERSISTENCE_VERSION or not valid_flag:
        return None

    if not math.isfinite(forward_yaw_rad):
        return None

    if not math.isfinite(uncertainty_forward_yaw_rad) or (
        uncertainty_forward_yaw_rad < 0.0
    ):
        return None

    return PersistedForwardYaw(
        forward_yaw_rad=forward_yaw_rad,
        uncertainty_forward_yaw_rad=uncertainty_forward_yaw_rad,
    )


def write_persisted_forward_yaw(
    *,
    path: str,
    host: str,
    created_unix_ns: int,
    forward_yaw_rad: float,
    uncertainty_forward_yaw_rad: float,
    metadata: PersistedForwardYawMetadata,
) -> None:
    """Atomically write one committed forward-yaw YAML file."""

    persistence_path: Path = Path(path).expanduser()
    persistence_path.parent.mkdir(parents=True, exist_ok=True)

    payload: dict[str, Any] = {
        "version": PERSISTENCE_VERSION,
        "created_unix_ns": int(created_unix_ns),
        "host": host,
        "estimator": "ahrs_speedometer",
        "valid": True,
        "forward_yaw_rad": float(forward_yaw_rad),
        "forward_axis": [
            math.cos(forward_yaw_rad),
            math.sin(forward_yaw_rad),
            0.0,
        ],
        "fit_sample_count": int(metadata.fit_sample_count),
        "checkpoint_count": int(metadata.checkpoint_count),
        "confidence": float(metadata.confidence),
        "residual": float(metadata.residual),
        "score": float(metadata.score),
        "uncertainty_forward_yaw_rad": float(uncertainty_forward_yaw_rad),
        "loaded_startup_capable": True,
        "last_update_reason": str(metadata.last_update_reason),
    }

    temporary_path: Path = persistence_path.with_suffix(
        persistence_path.suffix + f".tmp.{os.getpid()}"
    )
    temporary_path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )
    os.replace(temporary_path, persistence_path)


def _finite_float(value: Any) -> float:
    if value is None:
        return math.nan

    try:
        converted_value: float = float(value)
    except (TypeError, ValueError):
        return math.nan

    return converted_value
