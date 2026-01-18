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
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


@dataclass(frozen=True, slots=True)
class MagPacket:
    """Magnetometer measurement packet definition for the AHRS core.

    Data contract:
        t_meas_ns:
            Measurement timestamp in int nanoseconds since an arbitrary epoch
            used for exact time alignment
        frame_id:
            Sensor frame identifier for {M} used to label z_m
        z_m:
            Magnetometer measurement vector (tesla) in frame {M}
        R_m_raw:
            Raw measurement covariance matrix (tesla^2), full 3x3 SPD

    Determinism and edge cases:
        - Packets are immutable once created for deterministic replay
        - Covariances must be full SPD matrices and are never diagonalized
    """

    t_meas_ns: int
    frame_id: str
    z_m: list[float]
    R_m_raw: list[list[float]]

    def validate(self) -> None:
        """Validate packet fields and raise ValueError on failure."""
        if not self._is_int(self.t_meas_ns):
            raise ValueError("t_meas_ns must be int nanoseconds")
        if not self.frame_id:
            raise ValueError("frame_id must be a non-empty string")
        self._validate_vector("z_m", self.z_m, 3)
        self._validate_matrix("R_m_raw", self.R_m_raw, 3)
        if not LinearAlgebra.is_spd(self.R_m_raw):
            raise ValueError("R_m_raw must be SPD")

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-serializable dict representation."""
        return {
            "t_meas_ns": self.t_meas_ns,
            "frame_id": self.frame_id,
            "z_m": list(self.z_m),
            "R_m_raw": [list(row) for row in self.R_m_raw],
        }

    @staticmethod
    def _is_int(value: object) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    @staticmethod
    def _validate_vector(name: str, vector: Sequence[float], length: int) -> None:
        if len(vector) != length:
            raise ValueError(f"{name} must have length {length}")

    @staticmethod
    def _validate_matrix(
        name: str, matrix: Sequence[Sequence[float]], size: int
    ) -> None:
        if len(matrix) != size:
            raise ValueError(f"{name} must be {size}x{size}")
        for row in matrix:
            if len(row) != size:
                raise ValueError(f"{name} must be {size}x{size}")
