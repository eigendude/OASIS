################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Diagnostics types for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from oasis_control.localization.mounting.mounting_types.update_report import (
    UpdateReport,
)


@dataclass(frozen=True)
class Diagnostics:
    """Structured diagnostics bundle for mounting calibration.

    Attributes:
        t_meas_ns: Timestamp associated with the diagnostics in nanoseconds
        pipeline_state: Human-readable pipeline state
        update_report: Optional update report details
        segment_count: Number of segments processed
        keyframe_count: Number of keyframes tracked
        diversity_tilt_deg: Tilt diversity in degrees when available
        diversity_yaw_deg: Yaw diversity in degrees when available
        warnings: Warning messages
        errors: Error messages
        info: Informational messages
        dropped_packets: Count of dropped input packets
    """

    t_meas_ns: int
    pipeline_state: str
    update_report: UpdateReport | None
    segment_count: int
    keyframe_count: int
    diversity_tilt_deg: float | None
    diversity_yaw_deg: float | None
    warnings: tuple[str, ...] = ()
    errors: tuple[str, ...] = ()
    info: tuple[str, ...] = ()
    dropped_packets: int = 0

    def __post_init__(self) -> None:
        """Validate diagnostic fields."""
        if not isinstance(self.t_meas_ns, int) or isinstance(self.t_meas_ns, bool):
            raise ValueError("t_meas_ns must be an int")
        if not isinstance(self.pipeline_state, str):
            raise ValueError("pipeline_state must be a str")
        if not isinstance(self.segment_count, int) or isinstance(
            self.segment_count, bool
        ):
            raise ValueError("segment_count must be an int")
        if self.segment_count < 0:
            raise ValueError("segment_count must be non-negative")
        if not isinstance(self.keyframe_count, int) or isinstance(
            self.keyframe_count, bool
        ):
            raise ValueError("keyframe_count must be an int")
        if self.keyframe_count < 0:
            raise ValueError("keyframe_count must be non-negative")
        if not isinstance(self.dropped_packets, int) or isinstance(
            self.dropped_packets, bool
        ):
            raise ValueError("dropped_packets must be an int")
        if self.dropped_packets < 0:
            raise ValueError("dropped_packets must be non-negative")

        if self.diversity_tilt_deg is not None:
            _require_finite(self.diversity_tilt_deg, "diversity_tilt_deg")
        if self.diversity_yaw_deg is not None:
            _require_finite(self.diversity_yaw_deg, "diversity_yaw_deg")

        warnings: tuple[str, ...] = _as_str_tuple(self.warnings, "warnings")
        errors: tuple[str, ...] = _as_str_tuple(self.errors, "errors")
        info: tuple[str, ...] = _as_str_tuple(self.info, "info")

        object.__setattr__(self, "warnings", warnings)
        object.__setattr__(self, "errors", errors)
        object.__setattr__(self, "info", info)

    def has_errors(self) -> bool:
        """Return True when any error messages are present."""
        return len(self.errors) > 0

    def to_dict(self) -> dict[str, object]:
        """Return a dictionary representation of the diagnostics."""
        data: dict[str, object] = {
            "t_meas_ns": self.t_meas_ns,
            "pipeline_state": self.pipeline_state,
            "segment_count": self.segment_count,
            "keyframe_count": self.keyframe_count,
            "diversity_tilt_deg": self.diversity_tilt_deg,
            "diversity_yaw_deg": self.diversity_yaw_deg,
            "warnings": self.warnings,
            "errors": self.errors,
            "info": self.info,
            "dropped_packets": self.dropped_packets,
        }
        if self.update_report is not None:
            data["update_report"] = self.update_report.to_dict()
        return data


def _require_finite(value: float, name: str) -> None:
    """Ensure a scalar value is finite."""
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _as_str_tuple(values: tuple[str, ...] | list[str], name: str) -> tuple[str, ...]:
    """Normalize a sequence of strings into an immutable tuple."""
    items: tuple[str, ...] = tuple(values)
    for item in items:
        if not isinstance(item, str):
            raise ValueError(f"{name} entries must be strings")
    return items
