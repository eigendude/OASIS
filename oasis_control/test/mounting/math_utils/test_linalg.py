################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for SO(3) linear algebra utilities."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.linalg import SO3
from oasis_control.localization.mounting.math_utils.linalg import Linalg


def test_hat_vee_roundtrip() -> None:
    """Checks hat/vee are inverses."""
    vec: NDArray[np.float64] = np.array([0.2, -0.1, 0.3], dtype=float)
    mat: NDArray[np.float64] = SO3.hat(vec)
    roundtrip: NDArray[np.float64] = SO3.vee(mat)
    assert np.allclose(roundtrip, vec)


def test_exp_log_roundtrip_small() -> None:
    """Checks exp/log roundtrip for small rotations."""
    rng: np.random.Generator = np.random.default_rng(0)
    vec: NDArray[np.float64] = rng.normal(scale=0.05, size=3)
    R: NDArray[np.float64] = SO3.exp(vec)
    roundtrip: NDArray[np.float64] = SO3.log(R)
    assert np.allclose(roundtrip, vec, atol=1e-6)


def test_project_to_so3() -> None:
    """Checks projection to SO(3) restores det=+1."""
    R: NDArray[np.float64] = SO3.exp(np.array([0.1, 0.2, -0.1], dtype=float))
    R_noisy: NDArray[np.float64] = R.copy()
    R_noisy[0, 0] += 0.01
    R_proj: NDArray[np.float64] = SO3.project_to_so3(R_noisy)
    det: float = float(np.linalg.det(R_proj))
    assert np.isclose(det, 1.0)
    assert np.allclose(R_proj.T @ R_proj, np.eye(3), atol=1e-6)


def test_angle_known_rotations() -> None:
    """Checks rotation angle extraction for a known rotation."""
    vec: NDArray[np.float64] = np.array([0.0, 0.0, np.pi / 2.0], dtype=float)
    R: NDArray[np.float64] = SO3.exp(vec)
    angle: float = SO3.angle(R)
    assert np.isclose(angle, np.pi / 2.0)


def test_block_diag() -> None:
    """Checks block diagonal assembly sizes."""
    A: NDArray[np.float64] = np.eye(2, dtype=float)
    B: NDArray[np.float64] = np.ones((1, 3), dtype=float)
    result: NDArray[np.float64] = Linalg.block_diag(A, B)
    assert result.shape == (3, 5)
    assert np.allclose(result[:2, :2], A)
    assert np.allclose(result[2:, 2:], B)
