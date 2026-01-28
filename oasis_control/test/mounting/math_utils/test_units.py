################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for units helpers."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.units import Angle
from oasis_control.localization.mounting.math_utils.units import PhysicalConstants
from oasis_control.localization.mounting.math_utils.units import assert_finite


def test_deg_rad_roundtrip_scalar() -> None:
    """Checks scalar deg/rad conversion roundtrip."""
    value_deg: float = 45.0
    value_rad: float = float(Angle.deg2rad(value_deg))
    roundtrip: float = float(Angle.rad2deg(value_rad))
    assert np.isclose(roundtrip, value_deg)


def test_deg_rad_roundtrip_array() -> None:
    """Checks array deg/rad conversion roundtrip."""
    values_deg: NDArray[np.float64] = np.array([0.0, 90.0, 180.0], dtype=float)
    values_rad: NDArray[np.float64] = np.asarray(Angle.deg2rad(values_deg), dtype=float)
    roundtrip: NDArray[np.float64] = np.asarray(Angle.rad2deg(values_rad), dtype=float)
    assert np.allclose(roundtrip, values_deg)


def test_constants_finite() -> None:
    """Ensures constants are finite."""
    gravity: float = PhysicalConstants.GRAVITY_MPS2
    eps: float = PhysicalConstants.EPS
    assert np.isfinite(gravity)
    assert np.isfinite(eps)


def test_assert_finite() -> None:
    """Ensures assert_finite rejects non-finite inputs."""
    good: NDArray[np.float64] = np.array([1.0, 2.0, 3.0], dtype=float)
    assert_finite(good, "good")
    bad: NDArray[np.float64] = np.array([1.0, np.nan, 3.0], dtype=float)
    with pytest.raises(ValueError):
        assert_finite(bad, "bad")
