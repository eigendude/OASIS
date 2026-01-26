################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Tests for SE(3) transforms."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.math_utils.se3 import SE3


def test_identity_inverse() -> None:
    """Checks identity and inverse composition."""
    identity: SE3 = SE3.identity()
    result: SE3 = identity * identity.inverse()
    assert np.allclose(result.R, np.eye(3))
    assert np.allclose(result.p, np.zeros(3))


def test_composition() -> None:
    """Checks transform composition behavior."""
    q1: Quaternion = Quaternion.from_rotvec(np.array([0.1, 0.0, 0.0], dtype=float))
    q2: Quaternion = Quaternion.from_rotvec(np.array([0.0, 0.2, 0.0], dtype=float))
    t1: SE3 = SE3.from_quat_translation(q1, np.array([1.0, 0.0, 0.0], dtype=float))
    t2: SE3 = SE3.from_quat_translation(q2, np.array([0.0, 1.0, 0.0], dtype=float))
    composed: SE3 = t1 * t2
    point: NDArray[np.float64] = np.array([0.5, -0.5, 2.0], dtype=float)
    direct: NDArray[np.float64] = t1.transform_point(t2.transform_point(point))
    assert np.allclose(composed.transform_point(point), direct)


def test_transform_point_matches_matrix() -> None:
    """Checks point transform matches homogeneous matrix."""
    q: Quaternion = Quaternion.from_rotvec(np.array([0.1, 0.1, 0.1], dtype=float))
    t: SE3 = SE3.from_quat_translation(q, np.array([1.0, 2.0, 3.0], dtype=float))
    point: NDArray[np.float64] = np.array([0.2, 0.3, -0.1], dtype=float)
    point_h: NDArray[np.float64] = np.hstack([point, 1.0])
    mat: NDArray[np.float64] = t.as_matrix4()
    expected: NDArray[np.float64] = (mat @ point_h)[:3]
    assert np.allclose(t.transform_point(point), expected)


def test_transform_vector_ignores_translation() -> None:
    """Checks vector transform ignores translation."""
    q: Quaternion = Quaternion.from_rotvec(np.array([0.0, 0.0, 0.2], dtype=float))
    t: SE3 = SE3.from_quat_translation(q, np.array([10.0, -5.0, 3.0], dtype=float))
    vec: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=float)
    assert np.allclose(t.transform_vector(vec), q.as_matrix() @ vec)
