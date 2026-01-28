################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Unit conversion helpers and physical constants."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


class Angle:
    """Angular unit conversions."""

    @staticmethod
    def deg2rad(x: float | NDArray[np.float64]) -> float | NDArray[np.float64]:
        """Convert degrees to radians."""
        arr: NDArray[np.float64] = np.asarray(x, dtype=float)
        result: NDArray[np.float64] = np.deg2rad(arr)
        if np.ndim(result) == 0:
            return float(result)
        return result

    @staticmethod
    def rad2deg(x: float | NDArray[np.float64]) -> float | NDArray[np.float64]:
        """Convert radians to degrees."""
        arr: NDArray[np.float64] = np.asarray(x, dtype=float)
        result: NDArray[np.float64] = np.rad2deg(arr)
        if np.ndim(result) == 0:
            return float(result)
        return result


class PhysicalConstants:
    """Physical constants used by math utilities."""

    GRAVITY_MPS2: float = 9.80665
    EPS: float = 1e-12


def assert_finite(x: NDArray[np.float64], name: str) -> None:
    """Raise ValueError when the array contains non-finite values."""
    if not np.all(np.isfinite(x)):
        raise ValueError(f"{name} must be finite")
