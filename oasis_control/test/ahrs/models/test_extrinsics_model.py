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
from typing import Tuple


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
from oasis_control.localization.ahrs.models.extrinsics_model import ExtrinsicsModel


class TestExtrinsicsModel(unittest.TestCase):
    """Tests for extrinsics model helpers."""

    def test_apply_delta_identity(self) -> None:
        """Zero delta leaves transform unchanged."""
        R: List[List[float]] = [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        p: List[float] = [1.0, 2.0, 3.0]
        T: Tuple[List[List[float]], List[float]] = (R, p)
        delta: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        T_updated: Tuple[List[List[float]], List[float]] = ExtrinsicsModel.apply_delta(
            T, delta
        )
        self.assertEqual(T_updated[0], R)
        self.assertEqual(T_updated[1], p)

    def test_jacobian_identity(self) -> None:
        """Jacobian with respect to delta is identity."""
        J: List[List[float]] = ExtrinsicsModel.jacobian_wrt_delta()
        self.assertEqual(len(J), 6)
        row: List[float]
        for row in J:
            self.assertEqual(len(row), 6)
        i: int
        j: int
        for i in range(6):
            for j in range(6):
                expected = 1.0 if i == j else 0.0
                self.assertEqual(J[i][j], expected)

    def test_adjoint_matches_se3(self) -> None:
        """Adjoint matches Se3.adjoint output."""
        R: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ]
        p: List[float] = [0.5, -0.25, 0.75]
        T: Tuple[List[List[float]], List[float]] = (R, p)
        self.assertEqual(ExtrinsicsModel.adjoint(T), Se3.adjoint(T))


if __name__ == "__main__":
    unittest.main()
