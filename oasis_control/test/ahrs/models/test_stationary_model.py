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

from oasis_control.localization.ahrs.models.stationary_model import StationaryModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


class TestStationaryModel(unittest.TestCase):
    """Tests for the StationaryModel residuals and Jacobians."""

    def test_zupt_residual(self) -> None:
        """ZUPT residual equals -v_WB for zero measurement."""
        state: AhrsState = AhrsState.reset()
        z_v: List[float] = [0.0, 0.0, 0.0]
        z_hat: List[float] = StationaryModel.predict_zupt(state)
        nu: List[float] = StationaryModel.residual_zupt(z_v, z_hat)
        expected: List[float] = [-state.v_WB[0], -state.v_WB[1], -state.v_WB[2]]
        self.assertEqual(nu, expected)

    def test_no_turn_residual(self) -> None:
        """No-turn residual equals -omega_WB for zero measurement."""
        state: AhrsState = AhrsState.reset()
        z_omega: List[float] = [0.0, 0.0, 0.0]
        z_hat: List[float] = StationaryModel.predict_no_turn(state)
        nu: List[float] = StationaryModel.residual_no_turn(z_omega, z_hat)
        expected: List[float] = [
            -state.omega_WB[0],
            -state.omega_WB[1],
            -state.omega_WB[2],
        ]
        self.assertEqual(nu, expected)

    def test_jacobian_slices(self) -> None:
        """Jacobians select δv and δω blocks only."""
        state: AhrsState = AhrsState.reset()
        H_zupt: List[List[float]] = StationaryModel.jacobian_zupt(state)
        delta_v: slice = StateMapping.slice_delta_v()
        for i in range(3):
            for j in range(StateMapping.dimension()):
                expected: float = 1.0 if j == delta_v.start + i else 0.0
                self.assertEqual(H_zupt[i][j], expected)

        H_no_turn: List[List[float]] = StationaryModel.jacobian_no_turn(state)
        delta_omega: slice = StateMapping.slice_delta_omega()
        for i in range(3):
            for j in range(StateMapping.dimension()):
                expected = 1.0 if j == delta_omega.start + i else 0.0
                self.assertEqual(H_no_turn[i][j], expected)


if __name__ == "__main__":
    unittest.main()
