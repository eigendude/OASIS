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
from typing import List, Sequence

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

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


def _matvec(A: Sequence[Sequence[float]], x: Sequence[float]) -> List[float]:
    """Multiply matrix A by vector x."""
    if len(A) == 0:
        raise ValueError("Matrix must be non-empty")
    if any(len(row) != len(x) for row in A):
        raise ValueError("Matrix must have columns matching vector")
    result: List[float] = [0.0 for _ in range(len(A))]
    for i, row in enumerate(A):
        acc: float = 0.0
        for j, value in enumerate(row):
            acc += value * x[j]
        result[i] = acc
    return result


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


class TestLinearAlgebra(unittest.TestCase):
    """Tests for the LinearAlgebra helper class."""

    def test_symmetrize(self) -> None:
        """symmetrize returns a symmetric matrix."""
        P: List[List[float]] = [[1.0, 2.0], [0.0, 3.0]]
        sym: List[List[float]] = LinearAlgebra.symmetrize(P)
        self.assertEqual(sym[0][1], sym[1][0])
        self.assertAlmostEqual(sym[0][1], 1.0)

    def test_cholesky_success_and_failure(self) -> None:
        """cholesky succeeds for SPD and fails for non-SPD."""
        spd: List[List[float]] = [[4.0, 1.0], [1.0, 3.0]]
        L: List[List[float]] = LinearAlgebra.cholesky(spd)
        self.assertGreater(L[0][0], 0.0)
        non_spd: List[List[float]] = [[1.0, 2.0], [2.0, 1.0]]
        with self.assertRaises(ValueError):
            LinearAlgebra.cholesky(non_spd)

    def test_solve_spd(self) -> None:
        """solve_spd returns a solution that satisfies A x = b."""
        A: List[List[float]] = [[4.0, 1.0], [1.0, 3.0]]
        b: List[float] = [1.0, 2.0]
        x: List[float] = LinearAlgebra.solve_spd(A, b)
        Ax: List[float] = _matvec(A, x)
        _assert_vec_close(self, Ax, b, tol=1e-9)

    def test_clamp_spd(self) -> None:
        """clamp_spd returns a symmetric positive definite matrix."""
        P: List[List[float]] = [
            [2.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 0.5],
        ]
        min_eig: float = 0.1
        max_eig: float = 3.0
        clamped: List[List[float]] = LinearAlgebra.clamp_spd(P, min_eig, max_eig)
        self.assertTrue(LinearAlgebra.is_spd(clamped))
        for i in range(3):
            self.assertGreaterEqual(clamped[i][i], min_eig)
            self.assertLessEqual(clamped[i][i], max_eig)


if __name__ == "__main__":
    unittest.main()
