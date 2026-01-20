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

from oasis_control.localization.ahrs.models.process_model import ProcessModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _identity(size: int) -> List[List[float]]:
    """Return identity matrix of given size."""
    return [[1.0 if i == j else 0.0 for j in range(size)] for i in range(size)]


class TestProcessModel(unittest.TestCase):
    """Tests for process model."""

    def test_random_walk_states_unchanged(self) -> None:
        """Random-walk states remain constant during propagation."""
        state: AhrsState = AhrsState(
            p_WB=[1.0, 2.0, 3.0],
            v_WB=[0.5, -0.5, 1.5],
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=[0.1, -0.2, 0.3],
            b_g=[0.01, 0.02, 0.03],
            b_a=[-0.1, 0.2, -0.3],
            A_a=_identity3(),
            T_BI=(_identity3(), [0.1, 0.2, 0.3]),
            T_BM=(_identity3(), [-0.1, -0.2, -0.3]),
            g_W=[0.0, 0.0, -9.81],
            m_W=[0.2, 0.3, 0.4],
        )
        propagated: AhrsState = ProcessModel.propagate_mean(state, 1_000_000_000)
        self.assertEqual(propagated.b_g, state.b_g)
        self.assertEqual(propagated.b_a, state.b_a)
        self.assertEqual(propagated.A_a, state.A_a)
        self.assertEqual(propagated.T_BI, state.T_BI)
        self.assertEqual(propagated.T_BM, state.T_BM)
        self.assertEqual(propagated.g_W, state.g_W)
        self.assertEqual(propagated.m_W, state.m_W)

    def test_jacobian_shapes(self) -> None:
        """Jacobian and discretization shapes are consistent."""
        state: AhrsState = AhrsState.reset()
        A: List[List[float]] = ProcessModel.linearize(state)
        G: List[List[float]] = ProcessModel.noise_jacobian(state)
        self.assertEqual(len(A), StateMapping.dimension())
        self.assertEqual(len(G), StateMapping.dimension())
        row: List[float]
        for row in A:
            self.assertEqual(len(row), StateMapping.dimension())
        for row in G:
            self.assertEqual(len(row), 39)
        Q_c: List[List[float]] = _identity(39)
        F: List[List[float]]
        Q: List[List[float]]
        F, Q = ProcessModel.discretize(A, G, Q_c, 1_000_000_000)
        self.assertEqual(len(F), StateMapping.dimension())
        self.assertEqual(len(Q), StateMapping.dimension())
        for row in F:
            self.assertEqual(len(row), StateMapping.dimension())
        for row in Q:
            self.assertEqual(len(row), StateMapping.dimension())

    def test_dt_ns_validation(self) -> None:
        """Non-positive dt_ns raises ValueError."""
        state: AhrsState = AhrsState.reset()
        with self.assertRaisesRegex(ValueError, "dt_ns must be positive"):
            ProcessModel.propagate_mean(state, 0)
        with self.assertRaisesRegex(ValueError, "dt_ns must be positive"):
            ProcessModel.discretize(
                ProcessModel.linearize(state),
                ProcessModel.noise_jacobian(state),
                _identity(39),
                -1,
            )


if __name__ == "__main__":
    unittest.main()
