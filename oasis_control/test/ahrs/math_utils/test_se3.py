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
from typing import List, Sequence, Tuple

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

from oasis_control.localization.ahrs.math_utils.se3 import Se3


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _assert_vec_close(
    testcase: unittest.TestCase,
    a: Sequence[float],
    b: Sequence[float],
    tol: float = 1e-8,
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
    tol: float = 1e-8,
) -> None:
    """Assert two matrices are close within tolerance."""
    testcase.assertEqual(len(A), len(B))
    for i in range(len(A)):
        testcase.assertEqual(len(A[i]), len(B[i]))
        for j in range(len(A[i])):
            testcase.assertLessEqual(abs(A[i][j] - B[i][j]), tol)


class TestSe3(unittest.TestCase):
    """Tests for SE(3) utilities."""

    def test_compose_inverse_identity(self) -> None:
        """compose(inverse(T), T) yields identity."""
        R: List[List[float]] = [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        p: List[float] = [1.0, 2.0, 3.0]
        T: Tuple[List[List[float]], List[float]] = (R, p)
        T_inv: Tuple[List[List[float]], List[float]] = Se3.inverse(T)
        T_id: Tuple[List[List[float]], List[float]] = Se3.compose(T_inv, T)
        _assert_mat_close(self, T_id[0], _identity3())
        _assert_vec_close(self, T_id[1], [0.0, 0.0, 0.0])

    def test_exp_log_round_trip(self) -> None:
        """exp/log round-trip for small perturbation."""
        delta_xi: List[float] = [0.1, -0.05, 0.02, 0.001, -0.002, 0.0005]
        T: Tuple[List[List[float]], List[float]] = Se3.exp(delta_xi)
        delta_back: List[float] = Se3.log(T)
        _assert_vec_close(self, delta_back, delta_xi, tol=1e-6)

    def test_adjoint_identity(self) -> None:
        """adjoint of identity has identity blocks."""
        T: Tuple[List[List[float]], List[float]] = (_identity3(), [0.0, 0.0, 0.0])
        Ad: List[List[float]] = Se3.adjoint(T)
        self.assertEqual(len(Ad), 6)
        for row in Ad:
            self.assertEqual(len(row), 6)
        for i in range(3):
            self.assertAlmostEqual(Ad[i][i], 1.0)
            self.assertAlmostEqual(Ad[i + 3][i + 3], 1.0)
        for i in range(3):
            for j in range(3, 6):
                self.assertAlmostEqual(Ad[i][j], 0.0)


if __name__ == "__main__":
    unittest.main()
