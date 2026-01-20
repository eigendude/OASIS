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
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True, slots=True)
class UpdateReport:
    """Update report definition for AHRS measurement updates.

    Data contract:
        t_meas_ns:
            Measurement timestamp in int nanoseconds since an arbitrary epoch
        measurement_type:
            Non-empty string identifying the measurement source
        z:
            Raw measurement vector in the measurement frame
        z_hat:
            Predicted measurement vector in the measurement frame
        nu:
            Residual vector nu = z - z_hat
        R:
            Full measurement covariance matrix, m x m
        S_hat:
            Optional predicted innovation covariance H P H^T, m x m
        S:
            Innovation covariance matrix S_hat + R, m x m
        mahalanobis2:
            Scalar nu^T S^-1 nu, must be finite and non-negative
        accepted:
            True when the update was applied
        rejection_reason:
            Deterministic non-empty string when the update is rejected

    Determinism and edge cases:
        - accepted implies rejection_reason == ""
        - rejected implies rejection_reason is non-empty
        - Shapes are validated deterministically for consistent logs
    """

    t_meas_ns: int
    measurement_type: str
    z: list[float]
    z_hat: list[float]
    nu: list[float]
    R: list[list[float]]
    S_hat: list[list[float]] | None
    S: list[list[float]]
    mahalanobis2: float
    accepted: bool
    rejection_reason: str

    def validate(self) -> None:
        """Validate report fields and raise ValueError on failure."""
        if not self._is_int(self.t_meas_ns):
            raise ValueError("t_meas_ns must be int nanoseconds")
        if not self.measurement_type:
            raise ValueError("measurement_type must be a non-empty string")
        m: int = len(self.z)
        if len(self.z_hat) != m or len(self.nu) != m:
            raise ValueError("z, z_hat, and nu must have matching lengths")
        for value in self.z:
            if not math.isfinite(value):
                raise ValueError("z contains non-finite value")
        for value in self.z_hat:
            if not math.isfinite(value):
                raise ValueError("z_hat contains non-finite value")
        for value in self.nu:
            if not math.isfinite(value):
                raise ValueError("nu contains non-finite value")
        self._validate_matrix("R", self.R, m)
        self._validate_matrix("S", self.S, m)
        if self.S_hat is not None:
            self._validate_matrix("S_hat", self.S_hat, m)
        if not math.isfinite(self.mahalanobis2):
            raise ValueError("mahalanobis2 must be finite")
        if self.mahalanobis2 < 0.0:
            raise ValueError("mahalanobis2 must be >= 0")
        if self.accepted and self.rejection_reason:
            raise ValueError("accepted report must have empty rejection_reason")
        if not self.accepted and not self.rejection_reason:
            raise ValueError("rejected report must have rejection_reason")

    def summarize(self) -> str:
        """Return a deterministic summary string for logs."""
        summary: str = (
            f"{self.measurement_type} t={self.t_meas_ns} "
            f"accepted={self.accepted} mahalanobis2={self.mahalanobis2}"
        )
        if not self.accepted:
            summary = f"{summary} reason={self.rejection_reason}"
        return summary

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_meas_ns": self.t_meas_ns,
            "measurement_type": self.measurement_type,
            "z": list(self.z),
            "z_hat": list(self.z_hat),
            "nu": list(self.nu),
            "R": [list(row) for row in self.R],
            "S_hat": None if self.S_hat is None else [list(row) for row in self.S_hat],
            "S": [list(row) for row in self.S],
            "mahalanobis2": self.mahalanobis2,
            "accepted": self.accepted,
            "rejection_reason": self.rejection_reason,
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
