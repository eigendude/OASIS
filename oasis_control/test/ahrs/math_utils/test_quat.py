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

from oasis_control.localization.ahrs.math_utils.quat import Quaternion


def _is_close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol


class TestQuaternion(unittest.TestCase):
    def test_normalize_identity(self) -> None:
        q: List[float] = [1.0, 0.0, 0.0, 0.0]
        qn: List[float] = Quaternion.normalize(q)
        self.assertEqual(qn, q)

        q_neg: List[float] = [-1.0, 0.0, 0.0, 0.0]
        qn_neg: List[float] = Quaternion.normalize(q_neg)
        self.assertEqual(qn_neg, q)

    def test_multiply_identity(self) -> None:
        q: List[float] = Quaternion.normalize([0.5, 0.5, 0.5, 0.5])
        identity: List[float] = [1.0, 0.0, 0.0, 0.0]
        q_out: List[float] = Quaternion.multiply(identity, q)
        for i in range(4):
            self.assertTrue(_is_close(q_out[i], q[i], 1e-12))

    def test_to_matrix_identity(self) -> None:
        q: List[float] = [1.0, 0.0, 0.0, 0.0]
        mat: List[List[float]] = Quaternion.to_matrix(q)
        self.assertTrue(_is_close(mat[0][0], 1.0, 1e-12))
        self.assertTrue(_is_close(mat[1][1], 1.0, 1e-12))
        self.assertTrue(_is_close(mat[2][2], 1.0, 1e-12))

    def test_round_trip_matrix(self) -> None:
        q: List[float] = Quaternion.normalize([0.8, -0.1, 0.2, 0.3])
        mat: List[List[float]] = Quaternion.to_matrix(q)
        q_out: List[float] = Quaternion.from_matrix(mat)
        for i in range(4):
            self.assertTrue(_is_close(q_out[i], q[i], 1e-9))

    def test_integrate_zero_rate(self) -> None:
        q: List[float] = Quaternion.normalize([0.6, 0.2, -0.1, 0.1])
        omega: List[float] = [0.0, 0.0, 0.0]
        q_next: List[float] = Quaternion.integrate(q, omega, 0.1)
        for i in range(4):
            self.assertTrue(_is_close(q_next[i], q[i], 1e-12))


if __name__ == "__main__":
    unittest.main()
