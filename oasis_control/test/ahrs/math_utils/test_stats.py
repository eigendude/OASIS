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

from oasis_control.localization.ahrs.math_utils.stats import Statistics


def _is_close(a: float, b: float, tol: float) -> bool:
    return abs(a - b) <= tol


class TestStatistics(unittest.TestCase):
    def test_innovation_covariance(self) -> None:
        h: List[List[float]] = [[1.0, 0.0], [0.0, 1.0]]
        p: List[List[float]] = [[2.0, 0.5], [0.5, 1.0]]
        r: List[List[float]] = [[0.1, 0.0], [0.0, 0.2]]
        s: List[List[float]] = Statistics.innovation_covariance(h, p, r)
        self.assertTrue(_is_close(s[0][0], 2.1, 1e-12))
        self.assertTrue(_is_close(s[0][1], 0.5, 1e-12))
        self.assertTrue(_is_close(s[1][0], 0.5, 1e-12))
        self.assertTrue(_is_close(s[1][1], 1.2, 1e-12))

    def test_mahalanobis_squared_identity(self) -> None:
        nu: List[float] = [1.0, 2.0, 3.0]
        s: List[List[float]] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        d2: float = Statistics.mahalanobis_squared(nu, s)
        self.assertTrue(_is_close(d2, 14.0, 1e-12))

    def test_gating_passes(self) -> None:
        nu: List[float] = [1.0, 0.0]
        s: List[List[float]] = [[1.0, 0.0], [0.0, 1.0]]
        self.assertTrue(Statistics.gating_passes(nu, s, 1.5))
        self.assertFalse(Statistics.gating_passes(nu, s, 0.5))


if __name__ == "__main__":
    unittest.main()
