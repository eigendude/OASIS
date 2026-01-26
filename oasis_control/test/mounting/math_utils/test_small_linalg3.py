################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for small 3D linear algebra helpers."""

from __future__ import annotations

import numpy as np
import pytest
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.small_linalg3 import Mat3
from oasis_control.localization.mounting.math_utils.small_linalg3 import Vec3


def test_normalize_unit_norm() -> None:
    """Checks normalization yields unit norm."""
    vec: NDArray[np.float64] = np.array([3.0, 0.0, 4.0], dtype=float)
    result: NDArray[np.float64] = Vec3.normalize(vec)
    assert np.isclose(float(np.linalg.norm(result)), 1.0)


def test_normalize_zero_raises() -> None:
    """Checks normalize rejects zero vectors."""
    vec: NDArray[np.float64] = np.zeros(3, dtype=float)
    with pytest.raises(ValueError):
        Vec3.normalize(vec)


def test_angle_between_limits() -> None:
    """Checks angle between identical and opposite vectors."""
    u: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=float)
    v: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=float)
    w: NDArray[np.float64] = np.array([-1.0, 0.0, 0.0], dtype=float)
    assert np.isclose(Vec3.angle_between(u, v), 0.0)
    assert np.isclose(Vec3.angle_between(u, w), np.pi)


def test_is_spd() -> None:
    """Checks SPD detection for valid and invalid matrices."""
    spd: NDArray[np.float64] = np.array(
        [[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]], dtype=float
    )
    non_spd: NDArray[np.float64] = np.array(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=float
    )
    assert Mat3.is_spd(spd)
    assert not Mat3.is_spd(non_spd)


def test_clamp_spd_symmetry() -> None:
    """Checks eigenvalue clamping and symmetry."""
    mat: NDArray[np.float64] = np.array(
        [[3.0, 1.0, 0.0], [1.0, 0.5, 0.0], [0.0, 0.0, 0.1]], dtype=float
    )
    clamped: NDArray[np.float64] = Mat3.clamp_spd(mat, eig_min=0.2, eig_max=2.0)
    eigvals: NDArray[np.float64] = np.asarray(np.linalg.eigvalsh(clamped), dtype=float)
    assert np.all(eigvals >= 0.2 - 1e-12)
    assert np.all(eigvals <= 2.0 + 1e-12)
    assert np.allclose(clamped, clamped.T)
