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

from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _identity(size: int) -> List[List[float]]:
    """Return an identity matrix."""
    matrix: List[List[float]] = []
    i: int
    for i in range(size):
        row: List[float] = [0.0 for _ in range(size)]
        row[i] = 1.0
        matrix.append(row)
    return matrix


def _matmul(
    A: List[List[float]],
    B: List[List[float]],
) -> List[List[float]]:
    """Multiply two matrices."""
    rows: int = len(A)
    cols: int = len(B[0])
    inner: int = len(B)
    result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(rows)]
    i: int
    for i in range(rows):
        j: int
        for j in range(cols):
            acc: float = 0.0
            k: int
            for k in range(inner):
                acc += A[i][k] * B[k][j]
            result[i][j] = acc
    return result


def _transpose(A: List[List[float]]) -> List[List[float]]:
    """Transpose a matrix."""
    rows: int = len(A)
    cols: int = len(A[0])
    result: List[List[float]] = [[0.0 for _ in range(rows)] for _ in range(cols)]
    i: int
    for i in range(rows):
        j: int
        for j in range(cols):
            result[j][i] = A[i][j]
    return result


class TestAhrsCovariance(unittest.TestCase):
    """Tests for AhrsCovariance."""

    def test_symmetrize(self) -> None:
        """symmetrize returns symmetric matrix."""
        size: int = StateMapping.dimension()
        P: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        P[0][1] = 2.0
        P[1][0] = 0.0
        cov: AhrsCovariance = AhrsCovariance(P=P)
        sym: AhrsCovariance = cov.symmetrize()
        self.assertEqual(sym.P[0][1], sym.P[1][0])

    def test_from_matrix_symmetrizes(self) -> None:
        """from_matrix symmetrizes input."""
        size: int = StateMapping.dimension()
        P: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        P[2][3] = 4.0
        cov: AhrsCovariance = AhrsCovariance.from_matrix(P)
        self.assertEqual(cov.P[2][3], cov.P[3][2])

    def test_map_with_jacobian(self) -> None:
        """map_with_jacobian matches manual computation."""
        size: int = StateMapping.dimension()
        P: List[List[float]] = _identity(size)
        P[0][0] = 2.0
        P[1][1] = 3.0
        cov: AhrsCovariance = AhrsCovariance.from_matrix(P)
        J: List[List[float]] = [
            [1.0 if idx == 0 else 0.0 for idx in range(size)],
            [1.0 if idx == 1 else 0.0 for idx in range(size)],
        ]
        mapped: List[List[float]] = cov.map_with_jacobian(J)
        expected: List[List[float]] = _matmul(_matmul(J, P), _transpose(J))
        self.assertEqual(mapped, expected)

    def test_apply_adjoint(self) -> None:
        """apply_adjoint updates only the target block."""
        size: int = StateMapping.dimension()
        P: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        block_start: int = StateMapping.slice_delta_xi_BI().start or 0
        i: int
        for i in range(6):
            P[block_start + i][block_start + i] = float(i + 1)
        cov: AhrsCovariance = AhrsCovariance.from_matrix(P)
        Ad_T: List[List[float]] = _identity(6)
        Ad_T[0][0] = 2.0
        updated: AhrsCovariance = cov.apply_adjoint(
            Ad_T, StateMapping.slice_delta_xi_BI()
        )
        self.assertEqual(updated.P[block_start][block_start], 4.0)
        self.assertEqual(updated.P[0][0], 0.0)
        self.assertEqual(updated.P[1][1], 0.0)

    def test_validation(self) -> None:
        """Validation rejects bad input."""
        with self.assertRaises(ValueError):
            AhrsCovariance.from_matrix([[math.nan]])
        with self.assertRaises(ValueError):
            AhrsCovariance.from_matrix([[0.0, 1.0], [1.0, 0.0]])


if __name__ == "__main__":
    unittest.main()
