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

from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.error_state import AhrsErrorState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _build_delta() -> List[float]:
    """Build a deterministic delta_x vector."""
    delta_x: List[float] = [0.0 for _ in range(StateMapping.dimension())]
    delta_x[StateMapping.slice_delta_p()] = [1.0, -2.0, 3.0]
    delta_x[StateMapping.slice_delta_theta()] = [0.01, 0.0, 0.0]
    delta_x[StateMapping.slice_delta_g_W()] = [0.1, -0.1, 0.2]
    return delta_x


class TestAhrsErrorState(unittest.TestCase):
    """Tests for AhrsErrorState."""

    def test_zero(self) -> None:
        """zero() returns a 45-element zero vector."""
        state: AhrsErrorState = AhrsErrorState.zero()
        self.assertEqual(len(state.delta_x), 45)
        value: float
        for value in state.delta_x:
            self.assertEqual(value, 0.0)

    def test_round_trip(self) -> None:
        """from_vector and as_vector preserve values."""
        delta_x: List[float] = _build_delta()
        err: AhrsErrorState = AhrsErrorState.from_vector(delta_x)
        round_trip: List[float] = err.as_vector()
        self.assertEqual(round_trip, delta_x)

    def test_accessors_return_copies(self) -> None:
        """Accessors return correct slices and copies."""
        delta_x: List[float] = _build_delta()
        err: AhrsErrorState = AhrsErrorState.from_vector(delta_x)
        delta_p: List[float] = err.delta_p()
        self.assertEqual(delta_p, [1.0, -2.0, 3.0])
        delta_p[0] = 9.0
        self.assertNotEqual(err.delta_p()[0], 9.0)

    def test_validation(self) -> None:
        """from_vector rejects bad input."""
        with self.assertRaises(ValueError):
            AhrsErrorState.from_vector([0.0])
        bad: List[float] = [0.0 for _ in range(StateMapping.dimension())]
        bad[0] = math.nan
        with self.assertRaises(ValueError):
            AhrsErrorState.from_vector(bad)

    def test_apply_to_state(self) -> None:
        """apply_to_state uses state.apply_error."""
        state: AhrsState = AhrsState.reset()
        delta_x: List[float] = _build_delta()
        err: AhrsErrorState = AhrsErrorState.from_vector(delta_x)
        updated: AhrsState = err.apply_to_state(state)
        self.assertEqual(updated.p_WB, [1.0, -2.0, 3.0])
        self.assertTrue(math.isfinite(updated.q_WB[0]))
        self.assertEqual(updated.g_W[:2], [0.1, -0.1])
        self.assertAlmostEqual(updated.g_W[2], -9.61, places=9)


if __name__ == "__main__":
    unittest.main()
