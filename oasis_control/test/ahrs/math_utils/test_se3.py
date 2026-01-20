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
import unittest
from typing import List
from typing import Tuple

from oasis_control.localization.ahrs.math_utils.se3 import Se3


def _identity3() -> List[List[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _is_close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol


class TestSe3(unittest.TestCase):
    def test_compose_inverse_identity(self) -> None:
        angle: float = math.pi / 4.0
        c: float = math.cos(angle)
        s: float = math.sin(angle)
        r: List[List[float]] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
        p: List[float] = [1.0, 2.0, -0.5]
        t: Tuple[List[List[float]], List[float]] = (r, p)
        t_inv: Tuple[List[List[float]], List[float]] = Se3.inverse(t)
        t_eye: Tuple[List[List[float]], List[float]] = Se3.compose(t_inv, t)
        r_eye: List[List[float]] = t_eye[0]
        p_eye: List[float] = t_eye[1]
        for i in range(3):
            for j in range(3):
                self.assertTrue(_is_close(r_eye[i][j], _identity3()[i][j], 1e-9))
        for i in range(3):
            self.assertTrue(_is_close(p_eye[i], 0.0, 1e-9))

    def test_exp_log_round_trip_small(self) -> None:
        delta_xi: List[float] = [0.01, -0.02, 0.03, 1e-4, -2e-4, 1.5e-4]
        t: Tuple[List[List[float]], List[float]] = Se3.exp(delta_xi)
        delta_out: List[float] = Se3.log(t)
        for i in range(6):
            self.assertTrue(_is_close(delta_out[i], delta_xi[i], 1e-6))

    def test_adjoint_identity(self) -> None:
        r_eye: List[List[float]] = _identity3()
        p_zero: List[float] = [0.0, 0.0, 0.0]
        adj: List[List[float]] = Se3.adjoint((r_eye, p_zero))
        self.assertEqual(len(adj), 6)
        self.assertEqual(len(adj[0]), 6)
        for i in range(3):
            for j in range(3):
                self.assertTrue(_is_close(adj[i][j], r_eye[i][j], 1e-12))
                self.assertTrue(_is_close(adj[i + 3][j + 3], r_eye[i][j], 1e-12))


if __name__ == "__main__":
    unittest.main()
