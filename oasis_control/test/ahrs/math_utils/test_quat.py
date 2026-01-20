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
import sys
import unittest
from pathlib import Path
from typing import List
from typing import Sequence


ROOT: Path = Path(__file__).resolve().parents[4]
PACKAGE_ROOT: Path = ROOT / "oasis_control"
PACKAGE_SRC: Path = PACKAGE_ROOT / "oasis_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
if "oasis_control" in sys.modules:
    module_path_raw = getattr(sys.modules["oasis_control"], "__path__", None)
    module_paths: List[str] = []
    if module_path_raw is not None:
        module_paths = list(module_path_raw)
        if str(PACKAGE_SRC) not in module_paths:
            module_paths.append(str(PACKAGE_SRC))
            sys.modules["oasis_control"].__path__ = module_paths

from oasis_control.localization.ahrs.math_utils.quat import Quaternion


def _assert_vec_close(
    testcase: unittest.TestCase,
    a: Sequence[float],
    b: Sequence[float],
    tol: float = 1e-9,
) -> None:
    """Assert two vectors are close within tolerance."""
    testcase.assertEqual(len(a), len(b))
    for i in range(len(a)):
        testcase.assertTrue(math.isfinite(a[i]))
        testcase.assertTrue(math.isfinite(b[i]))
        testcase.assertLessEqual(abs(a[i] - b[i]), tol)


def _assert_mat_close(
    testcase: unittest.TestCase,
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
    tol: float = 1e-9,
) -> None:
    """Assert two matrices are close within tolerance."""
    testcase.assertEqual(len(A), len(B))
    for i in range(len(A)):
        testcase.assertEqual(len(A[i]), len(B[i]))
        for j in range(len(A[i])):
            testcase.assertLessEqual(abs(A[i][j] - B[i][j]), tol)


class TestQuaternion(unittest.TestCase):
    """Tests for quaternion math utilities."""

    def test_normalize_identity_and_canonical(self) -> None:
        """Normalize handles identity and canonical sign."""
        q: List[float] = [1.0, 0.0, 0.0, 0.0]
        qn: List[float] = Quaternion.normalize(q)
        _assert_vec_close(self, qn, q)
        q_neg: List[float] = [-1.0, 0.0, 0.0, 0.0]
        qn_neg: List[float] = Quaternion.normalize(q_neg)
        _assert_vec_close(self, qn_neg, q)

    def test_multiply_identity(self) -> None:
        """Multiply with identity returns the other quaternion."""
        q: List[float] = Quaternion.normalize([0.5, 0.2, -0.1, 0.1])
        identity: List[float] = [1.0, 0.0, 0.0, 0.0]
        result: List[float] = Quaternion.multiply(identity, q)
        _assert_vec_close(self, result, q)

    def test_to_matrix_identity(self) -> None:
        """Test that to_matrix produces identity for identity quaternion."""
        q: List[float] = [1.0, 0.0, 0.0, 0.0]
        R: List[List[float]] = Quaternion.to_matrix(q)
        expected: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        _assert_mat_close(self, R, expected)

    def test_round_trip_matrix(self) -> None:
        """Test that from_matrix(to_matrix(q)) returns the same rotation."""
        q: List[float] = Quaternion.normalize([0.7, -0.1, 0.2, 0.1])
        R: List[List[float]] = Quaternion.to_matrix(q)
        q_back: List[float] = Quaternion.from_matrix(R)
        _assert_vec_close(self, q_back, q, tol=1e-8)

    def test_integrate_zero_rate(self) -> None:
        """Integrate leaves quaternion unchanged for zero omega."""
        q: List[float] = Quaternion.normalize([0.7, 0.1, 0.2, -0.1])
        omega: List[float] = [0.0, 0.0, 0.0]
        q_next: List[float] = Quaternion.integrate(q, omega, 0.1)
        _assert_vec_close(self, q_next, q, tol=1e-9)


if __name__ == "__main__":
    unittest.main()
