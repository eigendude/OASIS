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

from oasis_control.localization.ahrs.math_utils.stats import Statistics


class TestStatistics(unittest.TestCase):
    """Tests for statistical utilities."""

    def test_innovation_covariance(self) -> None:
        """innovation_covariance matches manual computation."""
        H: List[List[float]] = [[1.0, 2.0], [0.0, 1.0]]
        P: List[List[float]] = [[2.0, 0.5], [0.5, 1.0]]
        R: List[List[float]] = [[0.1, 0.0], [0.0, 0.2]]
        S: List[List[float]] = Statistics.innovation_covariance(H, P, R)
        expected: List[List[float]] = [[8.1, 2.5], [2.5, 1.2]]
        self.assertAlmostEqual(S[0][0], expected[0][0])
        self.assertAlmostEqual(S[0][1], expected[0][1])
        self.assertAlmostEqual(S[1][0], expected[1][0])
        self.assertAlmostEqual(S[1][1], expected[1][1])

    def test_mahalanobis_squared_identity(self) -> None:
        """mahalanobis_squared returns squared norm for identity S."""
        nu: List[float] = [1.0, -2.0, 0.5]
        S: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        d2: float = Statistics.mahalanobis_squared(nu, S)
        self.assertAlmostEqual(d2, 1.0 + 4.0 + 0.25)

    def test_gating_passes(self) -> None:
        """gating_passes respects the provided threshold."""
        nu: List[float] = [1.0, 0.0]
        S: List[List[float]] = [[1.0, 0.0], [0.0, 1.0]]
        self.assertTrue(Statistics.gating_passes(nu, S, threshold=1.0))
        self.assertFalse(Statistics.gating_passes(nu, S, threshold=0.5))


if __name__ == "__main__":
    unittest.main()
