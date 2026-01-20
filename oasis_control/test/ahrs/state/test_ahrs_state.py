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
from typing import Sequence


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

from oasis_control.localization.ahrs.math_utils.quat import Quaternion
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _assert_vec_close(
    testcase: unittest.TestCase,
    a: Sequence[float],
    b: Sequence[float],
    tol: float = 1e-9,
) -> None:
    """Assert two vectors are close."""
    testcase.assertEqual(len(a), len(b))
    idx: int
    for idx in range(len(a)):
        testcase.assertLessEqual(abs(a[idx] - b[idx]), tol)


class TestAhrsState(unittest.TestCase):
    """Tests for AhrsState."""

    def test_reset_defaults(self) -> None:
        """reset() returns deterministic defaults."""
        state: AhrsState = AhrsState.reset()
        self.assertEqual(state.p_WB, [0.0, 0.0, 0.0])
        self.assertEqual(state.v_WB, [0.0, 0.0, 0.0])
        self.assertEqual(state.q_WB, [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(
            state.A_a,
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        )
        self.assertEqual(state.T_BI[1], [0.0, 0.0, 0.0])
        self.assertEqual(state.g_W, [0.0, 0.0, -9.81])
        self.assertEqual(state.m_W, [1.0, 0.0, 0.0])

    def test_apply_error_updates(self) -> None:
        """apply_error updates state blocks."""
        state: AhrsState = AhrsState.reset()
        delta_x: List[float] = [0.0 for _ in range(StateMapping.dimension())]
        delta_x[StateMapping.slice_delta_p()] = [0.5, -0.5, 1.0]
        delta_x[StateMapping.slice_delta_v()] = [1.0, 2.0, 3.0]
        delta_x[StateMapping.slice_delta_theta()] = [0.01, 0.0, 0.0]
        delta_x[StateMapping.slice_delta_A_a()] = [0.1 for _ in range(9)]
        delta_x[StateMapping.slice_delta_g_W()] = [0.0, 0.0, 0.19]

        updated: AhrsState = state.apply_error(delta_x)
        self.assertEqual(updated.p_WB, [0.5, -0.5, 1.0])
        self.assertEqual(updated.v_WB, [1.0, 2.0, 3.0])
        self.assertEqual(updated.g_W[:2], [0.0, 0.0])
        self.assertAlmostEqual(updated.g_W[2], -9.62, places=9)
        self.assertEqual(updated.A_a[0][0], 1.1)

        dq: List[float] = Quaternion.small_angle_quat([0.01, 0.0, 0.0])
        expected_q: List[float] = Quaternion.multiply(dq, state.q_WB)
        _assert_vec_close(self, updated.q_WB, expected_q, tol=1e-8)
        q_norm: float = math.sqrt(sum(value * value for value in updated.q_WB))
        self.assertAlmostEqual(q_norm, 1.0, places=9)

    def test_round_trip_vector(self) -> None:
        """from_vector and as_vector round-trip."""
        state: AhrsState = AhrsState.reset()
        packed: List[float] = state.as_vector()
        rebuilt: AhrsState = AhrsState.from_vector(packed)
        self.assertEqual(rebuilt.as_vector(), packed)

    def test_validation(self) -> None:
        """Validation rejects wrong sizes and non-finite values."""
        with self.assertRaises(ValueError):
            AhrsState(
                p_WB=[0.0, 0.0],
                v_WB=[0.0, 0.0, 0.0],
                q_WB=[1.0, 0.0, 0.0, 0.0],
                omega_WB=[0.0, 0.0, 0.0],
                b_g=[0.0, 0.0, 0.0],
                b_a=[0.0, 0.0, 0.0],
                A_a=[
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                T_BI=(
                    [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    [0.0, 0.0, 0.0],
                ),
                T_BM=(
                    [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    [0.0, 0.0, 0.0],
                ),
                g_W=[0.0, 0.0, -9.81],
                m_W=[1.0, 0.0, 0.0],
            )
        with self.assertRaises(ValueError):
            AhrsState.from_vector([math.nan for _ in range(58)])


if __name__ == "__main__":
    unittest.main()
