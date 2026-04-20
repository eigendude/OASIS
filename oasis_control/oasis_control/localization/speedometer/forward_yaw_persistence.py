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
from pathlib import Path
from typing import Final
from typing import cast

import yaml  # type: ignore[import-untyped]

from oasis_control.localization.speedometer.contracts import PersistenceRecord


################################################################################
# Persistence
################################################################################


DEFAULT_MOUNT_INFO_DIRECTORY: Final[Path] = Path.home() / ".ros" / "mount_info"


class ForwardYawPersistence:
    """
    YAML reader and writer for committed forward-yaw checkpoint state.
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
                    "confidence": float(record.confidence),
                    "score": float(record.score),
                    "residual": float(record.residual),
                    "uncertainty_forward_yaw_rad": float(
                        record.uncertainty_forward_yaw_rad
                    ),
                    "loaded_startup_capable": bool(record.loaded_startup_capable),
                    "last_update_reason": record.last_update_reason,
                },
                output_file,
                sort_keys=True,
            )
        os.replace(temporary_path, self.path)

    def load(self) -> PersistenceRecord:
        """Load one committed forward-yaw payload from YAML."""

        with self.path.open("r", encoding="utf-8") as input_file:
            payload: object = yaml.safe_load(input_file)

        if not isinstance(payload, dict):
            raise ValueError("persistence payload is not a mapping")

        payload_dict: dict[str, object] = cast(dict[str, object], payload)
        forward_axis_object: object = payload_dict.get("forward_axis")
        if not isinstance(forward_axis_object, list) or len(forward_axis_object) != 3:
            raise ValueError("forward_axis must be a 3-element list")

        forward_axis_xyz: tuple[float, float, float] = (
            _as_float(forward_axis_object[0]),
            _as_float(forward_axis_object[1]),
            _as_float(forward_axis_object[2]),
        )
        forward_yaw_rad: float = _as_float(payload_dict["forward_yaw_rad"])
        fit_sample_count: int = _as_int(payload_dict.get("fit_sample_count", 0))
        checkpoint_count: int = _as_int(payload_dict.get("checkpoint_count", 0))
        confidence: float = _derive_confidence(
            payload_dict=payload_dict,
            fit_sample_count=fit_sample_count,
        )
        residual: float = _derive_residual(
            payload_dict=payload_dict,
            confidence=confidence,
        )
        score: float = _derive_score(
            payload_dict=payload_dict,
            fit_sample_count=fit_sample_count,
            confidence=confidence,
        )
        uncertainty_forward_yaw_rad: float = _derive_uncertainty_forward_yaw_rad(
            payload_dict=payload_dict,
            fit_sample_count=fit_sample_count,
            residual=residual,
        )

        record: PersistenceRecord = PersistenceRecord(
            version=_as_int(payload_dict.get("version", 1)),
            created_unix_ns=_as_int(payload_dict.get("created_unix_ns", 0)),
            hostname=str(payload_dict.get("host", self._hostname)),
            estimator=str(payload_dict.get("estimator", "ahrs_forward_twist")),
            valid=bool(payload_dict.get("valid", True)),
            forward_yaw_rad=forward_yaw_rad,
            forward_axis_xyz=forward_axis_xyz,
            fit_sample_count=fit_sample_count,
            checkpoint_count=checkpoint_count,
            confidence=confidence,
            score=score,
            residual=residual,
            uncertainty_forward_yaw_rad=uncertainty_forward_yaw_rad,
            loaded_startup_capable=bool(
                payload_dict.get("loaded_startup_capable", True)
            ),
            last_update_reason=str(payload_dict.get("last_update_reason", "")),
        )
        _validate_loaded_record(record)
        return record


def _derive_confidence(
    *, payload_dict: dict[str, object], fit_sample_count: int
) -> float:
    """Return the persisted confidence or a compatible derived fallback."""

    if "confidence" in payload_dict:
        return max(0.0, min(1.0, _as_float(payload_dict["confidence"])))

    if fit_sample_count <= 0:
        return 0.0

    return max(0.5, min(0.995, 1.0 - 1.0 / (1.0 + float(fit_sample_count))))


def _derive_residual(*, payload_dict: dict[str, object], confidence: float) -> float:
    """Return the persisted residual or a compatible derived fallback."""

    if "residual" in payload_dict:
        return max(0.0, _as_float(payload_dict["residual"]))

    return max(0.0, 1.0 - confidence)


def _derive_score(
    *,
    payload_dict: dict[str, object],
    fit_sample_count: int,
    confidence: float,
) -> float:
    """Return the persisted score or a compatible derived fallback."""

    if "score" in payload_dict:
        return _as_float(payload_dict["score"])

    return confidence * math.log1p(max(0, fit_sample_count))


def _derive_uncertainty_forward_yaw_rad(
    *,
    payload_dict: dict[str, object],
    fit_sample_count: int,
    residual: float,
) -> float:
    """Return the persisted yaw uncertainty or a compatible derived fallback."""

    if "uncertainty_forward_yaw_rad" in payload_dict:
        return max(0.0, _as_float(payload_dict["uncertainty_forward_yaw_rad"]))

    sample_term: float = 1.0 / math.sqrt(max(1, fit_sample_count))
    return max(0.01, math.sqrt(max(0.0, residual)) * sample_term)


def _validate_loaded_record(record: PersistenceRecord) -> None:
    """Validate one loaded persistence record for startup use."""

    if record.valid is not True:
        raise ValueError("persisted forward-yaw record is marked invalid")
    if record.loaded_startup_capable is not True:
        raise ValueError("persisted forward-yaw record is not startup-capable")
    if record.fit_sample_count < 0 or record.checkpoint_count < 0:
        raise ValueError("persisted sample counts must be nonnegative")


def _as_float(value: object) -> float:
    """Convert one YAML scalar-like value to float."""

    return float(cast(float | int | str, value))


def _as_int(value: object) -> int:
    """Convert one YAML scalar-like value to int."""

    return int(cast(float | int | str, value))
