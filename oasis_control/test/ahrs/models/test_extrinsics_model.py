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

from oasis_control.localization.ahrs.models.extrinsics_model import ExtrinsicsModel


def _identity3() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


class TestExtrinsicsModel(unittest.TestCase):
    """Tests for the ExtrinsicsModel helpers."""

    def test_apply_delta_zero(self) -> None:
        """Zero perturbation keeps the transform unchanged."""
        T_bx: tuple[list[list[float]], list[float]] = (_identity3(), [0.0, 0.0, 0.0])
        delta_xi: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result: tuple[list[list[float]], list[float]] = ExtrinsicsModel.apply_delta(
            T_bx, delta_xi
        )
        self.assertEqual(result[0], T_bx[0])
        self.assertEqual(result[1], T_bx[1])

    def test_adjoint_identity(self) -> None:
        """Adjoint of identity transform is the 6x6 identity."""
        T_bx: tuple[list[list[float]], list[float]] = (_identity3(), [0.0, 0.0, 0.0])
        Ad: List[List[float]] = ExtrinsicsModel.adjoint(T_bx)
        self.assertEqual(len(Ad), 6)
        for row in Ad:
            self.assertEqual(len(row), 6)
        for i in range(6):
            for j in range(6):
                expected: float = 1.0 if i == j else 0.0
                self.assertEqual(Ad[i][j], expected)


if __name__ == "__main__":
    unittest.main()
