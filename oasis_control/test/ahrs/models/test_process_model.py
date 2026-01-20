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


class TestProcessModel(unittest.TestCase):
    """Tests for the ProcessModel implementation."""

    def test_propagate_mean_random_walk_unchanged(self) -> None:
        """Random-walk states stay constant through mean propagation."""
        state: AhrsState = AhrsState.reset()
        dt_ns: int = 1_000_000_000
        propagated: AhrsState = ProcessModel.propagate_mean(state, dt_ns)
        self.assertEqual(propagated.b_g, state.b_g)
        self.assertEqual(propagated.b_a, state.b_a)
        self.assertEqual(propagated.A_a, state.A_a)
        self.assertEqual(propagated.T_BI, state.T_BI)
        self.assertEqual(propagated.T_BM, state.T_BM)
        self.assertEqual(propagated.g_W, state.g_W)
        self.assertEqual(propagated.m_W, state.m_W)

    def test_discretize_shapes(self) -> None:
        """Discrete F and Q have expected shapes."""
        state: AhrsState = AhrsState.reset()
        A: List[List[float]] = ProcessModel.linearize(state)
        G: List[List[float]] = ProcessModel.noise_jacobian(state)
        q_dim: int = 39
        Q_c: List[List[float]] = [[0.0 for _ in range(q_dim)] for _ in range(q_dim)]
        for i in range(q_dim):
            Q_c[i][i] = 1.0
        dt_ns: int = 1_000_000
        F: List[List[float]]
        Q: List[List[float]]
        F, Q = ProcessModel.discretize(A, G, Q_c, dt_ns)
        self.assertEqual(len(F), StateMapping.dimension())
        self.assertEqual(len(F[0]), StateMapping.dimension())
        self.assertEqual(len(Q), StateMapping.dimension())
        self.assertEqual(len(Q[0]), StateMapping.dimension())

    def test_dt_ns_validation(self) -> None:
        """Invalid dt_ns raises a ValueError with a precise message."""
        state: AhrsState = AhrsState.reset()
        with self.assertRaisesRegex(
            ValueError, "dt_ns must be positive int nanoseconds"
        ):
            ProcessModel.propagate_mean(state, 0)


if __name__ == "__main__":
    unittest.main()
