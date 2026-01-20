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

import unittest
from typing import List

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


def _is_symmetric(matrix: List[List[float]], tol: float) -> bool:
    size: int = len(matrix)
    for i in range(size):
        for j in range(size):
            if abs(matrix[i][j] - matrix[j][i]) > tol:
                return False
    return True


class TestLinearAlgebra(unittest.TestCase):
    def test_symmetrize(self) -> None:
        matrix: List[List[float]] = [[1.0, 2.0], [3.0, 4.0]]
        sym: List[List[float]] = LinearAlgebra.symmetrize(matrix)
        self.assertTrue(_is_symmetric(sym, 1e-12))
        self.assertAlmostEqual(sym[0][1], 2.5, places=12)

    def test_cholesky_success(self) -> None:
        matrix: List[List[float]] = [[4.0, 1.0], [1.0, 3.0]]
        lower: List[List[float]] = LinearAlgebra.cholesky(matrix)
        self.assertGreater(lower[0][0], 0.0)
        self.assertGreater(lower[1][1], 0.0)

    def test_cholesky_failure(self) -> None:
        matrix: List[List[float]] = [[1.0, 2.0], [2.0, 1.0]]
        with self.assertRaises(ValueError):
            LinearAlgebra.cholesky(matrix)

    def test_solve_spd_vector(self) -> None:
        matrix: List[List[float]] = [[4.0, 1.0], [1.0, 3.0]]
        b: List[float] = [1.0, 2.0]
        x: List[float] = LinearAlgebra.solve_spd(matrix, b)  # type: ignore[assignment]
        self.assertAlmostEqual(x[0], 1.0 / 11.0, places=12)
        self.assertAlmostEqual(x[1], 7.0 / 11.0, places=12)

    def test_clamp_spd(self) -> None:
        matrix: List[List[float]] = [[-1.0, 0.2, 0.0], [0.2, 2.0, 0.0], [0.0, 0.0, 0.1]]
        clamped: List[List[float]] = LinearAlgebra.clamp_spd(matrix, 0.5, 2.0)
        self.assertTrue(_is_symmetric(clamped, 1e-9))
        self.assertTrue(LinearAlgebra.is_spd(clamped))
        self.assertGreater(clamped[0][0], 0.0)
        self.assertGreater(clamped[1][1], 0.0)
        self.assertGreater(clamped[2][2], 0.0)


if __name__ == "__main__":
    unittest.main()
