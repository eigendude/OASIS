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


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


class TestStationaryModel(unittest.TestCase):
    """Tests for stationary pseudo-measurements."""

    def test_zupt_residual(self) -> None:
        """ZUPT residual equals -v_WB."""
        state: AhrsState = AhrsState(
            p_WB=[0.0, 0.0, 0.0],
            v_WB=[1.0, -2.0, 3.0],
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=[0.0, 0.0, 0.0],
            b_g=[0.0, 0.0, 0.0],
            b_a=[0.0, 0.0, 0.0],
            A_a=_identity3(),
            T_BI=(_identity3(), [0.0, 0.0, 0.0]),
            T_BM=(_identity3(), [0.0, 0.0, 0.0]),
            g_W=[0.0, 0.0, -9.81],
            m_W=[1.0, 0.0, 0.0],
        )
        z_hat: List[float] = StationaryModel.predict_zupt(state)
        residual: List[float] = StationaryModel.residual_zupt([0.0, 0.0, 0.0], z_hat)
        self.assertEqual(residual, [-1.0, 2.0, -3.0])

    def test_no_turn_residual(self) -> None:
        """No-turn residual equals -omega_WB."""
        state: AhrsState = AhrsState(
            p_WB=[0.0, 0.0, 0.0],
            v_WB=[0.0, 0.0, 0.0],
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=[0.1, -0.2, 0.3],
            b_g=[0.0, 0.0, 0.0],
            b_a=[0.0, 0.0, 0.0],
            A_a=_identity3(),
            T_BI=(_identity3(), [0.0, 0.0, 0.0]),
            T_BM=(_identity3(), [0.0, 0.0, 0.0]),
            g_W=[0.0, 0.0, -9.81],
            m_W=[1.0, 0.0, 0.0],
        )
        z_hat: List[float] = StationaryModel.predict_no_turn(state)
        residual: List[float] = StationaryModel.residual_no_turn([0.0, 0.0, 0.0], z_hat)
        self.assertEqual(residual, [-0.1, 0.2, -0.3])

    def test_jacobians_select_blocks(self) -> None:
        """Jacobians select delta_v and delta_omega blocks."""
        state: AhrsState = AhrsState.reset()
        H_v: List[List[float]] = StationaryModel.jacobian_zupt(state)
        H_w: List[List[float]] = StationaryModel.jacobian_no_turn(state)
        v_slice: slice = StateMapping.slice_delta_v()
        omega_slice: slice = StateMapping.slice_delta_omega()
        i: int
        j: int
        for i in range(3):
            for j in range(StateMapping.dimension()):
                if v_slice.start <= j < v_slice.stop:
                    expected = 1.0 if (j - v_slice.start) == i else 0.0
                    self.assertEqual(H_v[i][j], expected)
                else:
                    self.assertEqual(H_v[i][j], 0.0)
                if omega_slice.start <= j < omega_slice.stop:
                    expected = 1.0 if (j - omega_slice.start) == i else 0.0
                    self.assertEqual(H_w[i][j], expected)
                else:
                    self.assertEqual(H_w[i][j], 0.0)


if __name__ == "__main__":
    unittest.main()
