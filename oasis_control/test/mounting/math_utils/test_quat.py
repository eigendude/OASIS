################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for quaternion utilities."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.linalg import SO3
from oasis_control.localization.mounting.math_utils.quat import Quaternion


def test_identity_properties() -> None:
    """Checks identity quaternion properties."""
    q: Quaternion = Quaternion.identity()
    mat: NDArray[np.float64] = q.as_matrix()
    assert np.allclose(mat, np.eye(3))


def test_multiplication_inverse() -> None:
    """Checks quaternion multiplication with inverse returns identity."""
    q: Quaternion = Quaternion.from_rotvec(np.array([0.2, 0.0, -0.1], dtype=float))
    identity: Quaternion = q * q.inverse()
    assert identity.almost_equal(Quaternion.identity())


def test_from_matrix_roundtrip() -> None:
    """Checks conversion between matrix and quaternion."""
    R: NDArray[np.float64] = SO3.exp(np.array([0.1, 0.2, 0.3], dtype=float))
    q: Quaternion = Quaternion.from_matrix(R)
    roundtrip: NDArray[np.float64] = q.as_matrix()
    assert np.allclose(roundtrip, R, atol=1e-8)


def test_from_rotvec_matches_exp() -> None:
    """Checks from_rotvec matches SO3.exp."""
    vec: NDArray[np.float64] = np.array([0.0, 0.0, 0.2], dtype=float)
    q: Quaternion = Quaternion.from_rotvec(vec)
    assert np.allclose(q.as_matrix(), SO3.exp(vec), atol=1e-8)


def test_rotate_matches_matrix() -> None:
    """Checks vector rotation matches matrix multiply."""
    vec: NDArray[np.float64] = np.array([1.0, 2.0, 3.0], dtype=float)
    q: Quaternion = Quaternion.from_rotvec(np.array([0.1, 0.2, 0.1], dtype=float))
    rotated: NDArray[np.float64] = q.rotate(vec)
    expected: NDArray[np.float64] = q.as_matrix() @ vec
    assert np.allclose(rotated, expected)


def test_almost_equal_sign_flip() -> None:
    """Checks almost_equal handles sign flips."""
    q: Quaternion = Quaternion.from_rotvec(np.array([0.1, -0.2, 0.1], dtype=float))
    q_neg: Quaternion = Quaternion(-q.wxyz)
    assert q.almost_equal(q_neg)
