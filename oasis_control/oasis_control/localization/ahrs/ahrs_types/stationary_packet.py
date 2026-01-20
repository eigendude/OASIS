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

from dataclasses import dataclass
from typing import Mapping
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


@dataclass(frozen=True, slots=True)
class StationaryPacket:
    """Stationary detection result packet for deterministic pseudo-updates.

    Data contract:
        t_meas_ns:
            Measurement timestamp in int nanoseconds and equal to window_end_ns
        window_start_ns:
            Inclusive window start in int nanoseconds for statistics
        window_end_ns:
            Inclusive window end in int nanoseconds for statistics
        is_stationary:
            Boolean decision for whether the window is stationary
        R_v:
            ZUPT covariance matrix (m/s)^2 in {W}, full 3x3 SPD
        R_omega:
            Optional no-turn covariance matrix (rad/s)^2 in {B}, full 3x3 SPD
        metadata:
            Deterministic JSON-serializable scores and thresholds

    Determinism and edge cases:
        - window_start_ns must be <= window_end_ns
        - t_meas_ns must equal window_end_ns for exact alignment
        - Covariances must be full SPD matrices and are never diagonalized
    """

    t_meas_ns: int
    window_start_ns: int
    window_end_ns: int
    is_stationary: bool
    R_v: list[list[float]]
    R_omega: list[list[float]] | None
    metadata: Mapping[str, object]

    def validate(self) -> None:
        """Validate packet fields and raise ValueError on failure."""
        if not self._is_int(self.t_meas_ns):
            raise ValueError("t_meas_ns must be int nanoseconds")
        if not self._is_int(self.window_start_ns):
            raise ValueError("window_start_ns must be int nanoseconds")
        if not self._is_int(self.window_end_ns):
            raise ValueError("window_end_ns must be int nanoseconds")
        if self.window_start_ns > self.window_end_ns:
            raise ValueError("window_start_ns must be <= window_end_ns")
        if self.t_meas_ns != self.window_end_ns:
            raise ValueError("t_meas_ns must equal window_end_ns")
        self._validate_matrix("R_v", self.R_v, 3)
        if not LinearAlgebra.is_spd(self.R_v):
            raise ValueError("R_v must be SPD")
        if self.R_omega is not None:
            self._validate_matrix("R_omega", self.R_omega, 3)
            if not LinearAlgebra.is_spd(self.R_omega):
                raise ValueError("R_omega must be SPD")
        if not isinstance(self.metadata, Mapping):
            raise ValueError("metadata must be a mapping")
        self._normalize_json_payload("metadata", self.metadata)

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        normalized_metadata: object = self._normalize_json_payload(
            "metadata",
            self.metadata,
        )
        return {
            "t_meas_ns": self.t_meas_ns,
            "window_start_ns": self.window_start_ns,
            "window_end_ns": self.window_end_ns,
            "is_stationary": self.is_stationary,
            "R_v": [list(row) for row in self.R_v],
            "R_omega": (
                None if self.R_omega is None else [list(row) for row in self.R_omega]
            ),
            "metadata": normalized_metadata,
        }

    @staticmethod
    def _is_int(value: object) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    @staticmethod
    def _validate_matrix(
        name: str, matrix: Sequence[Sequence[float]], size: int
    ) -> None:
        if len(matrix) != size:
            raise ValueError(f"{name} must be {size}x{size}")
        for row in matrix:
            if len(row) != size:
                raise ValueError(f"{name} must be {size}x{size}")

    @classmethod
    def _normalize_json_payload(cls, name: str, payload: object) -> object:
        if payload is None:
            return None
        if isinstance(payload, (str, int, float, bool)):
            return payload
        if isinstance(payload, Mapping):
            normalized: dict[str, object] = {}
            for key, value in payload.items():
                if not isinstance(key, str):
                    raise ValueError(f"{name} has non-string key")
                normalized[key] = cls._normalize_json_payload(name, value)
            return normalized
        if isinstance(payload, Sequence) and not isinstance(payload, (str, bytes)):
            return [cls._normalize_json_payload(name, item) for item in payload]
        raise ValueError(f"{name} is not JSON-serializable")
