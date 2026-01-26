################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Magnetometer packet types for mounting calibration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class MagPacket:
    """Magnetometer sample with covariance."""

    t_meas_ns: int
    frame_id: str
    m_raw_T: np.ndarray
    cov_m_raw_T2: np.ndarray

    def __post_init__(self) -> None:
        """Validate packet fields and coerce arrays."""
        if not isinstance(self.t_meas_ns, int) or isinstance(self.t_meas_ns, bool):
            raise ValueError("t_meas_ns must be an int")
        if not isinstance(self.frame_id, str):
            raise ValueError("frame_id must be a str")

        m_raw_T: np.ndarray = _as_float_array(self.m_raw_T, "m_raw_T", (3,))
        cov_m_raw_T2: np.ndarray = _as_float_array(
            self.cov_m_raw_T2,
            "cov_m_raw_T2",
            (3, 3),
        )

        object.__setattr__(self, "m_raw_T", m_raw_T)
        object.__setattr__(self, "cov_m_raw_T2", cov_m_raw_T2)

    def magnitude_T(self) -> float:
        """Return the magnitude of the raw magnetic field in tesla."""
        magnitude: float = float(np.linalg.norm(self.m_raw_T))
        return magnitude

    def unit_direction(self) -> np.ndarray:
        """Return the unit magnetic field direction without mutating state."""
        return _unit_vector(self.m_raw_T, "m_raw_T")


def _as_float_array(value: Any, name: str, shape: tuple[int, ...]) -> np.ndarray:
    """Coerce a value to a float64 numpy array with a specific shape."""
    array: np.ndarray = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array


def _unit_vector(vector: np.ndarray, name: str) -> np.ndarray:
    """Return a unit vector derived from the input."""
    norm: float = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{name} must be non-zero to normalize")
    return vector / norm
