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

from oasis_control.localization.ahrs.models.mag_model import MagModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _matvec3(A: List[List[float]], v: List[float]) -> List[float]:
    """Return A * v for 3x3 A and 3x1 v."""
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


class TestMagModel(unittest.TestCase):
    """Tests for magnetometer model."""

    def test_residual_sign(self) -> None:
        """Residual follows ν = z - z_hat."""
        z_m: List[float] = [1.0, 2.0, 3.0]
        z_hat: List[float] = [0.5, 1.5, 2.5]
        self.assertEqual(MagModel.residual(z_m, z_hat), [0.5, 0.5, 0.5])

    def test_predict_identity_chain(self) -> None:
        """Identity rotations return m_W in {M}."""
        m_W: List[float] = [0.1, -0.2, 0.3]
        state: AhrsState = AhrsState(
            p_WB=[0.0, 0.0, 0.0],
            v_WB=[0.0, 0.0, 0.0],
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=[0.0, 0.0, 0.0],
            b_g=[0.0, 0.0, 0.0],
            b_a=[0.0, 0.0, 0.0],
            A_a=_identity3(),
            T_BI=(_identity3(), [0.0, 0.0, 0.0]),
            T_BM=(_identity3(), [0.0, 0.0, 0.0]),
            g_W=[0.0, 0.0, -9.81],
            m_W=m_W,
        )
        self.assertEqual(MagModel.predict(state), m_W)

    def test_jacobian_structure(self) -> None:
        """Jacobian only touches δθ and δm_W blocks."""
        state: AhrsState = AhrsState(
            p_WB=[0.0, 0.0, 0.0],
            v_WB=[0.0, 0.0, 0.0],
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
        H: List[List[float]] = MagModel.jacobian(state)
        self.assertEqual(len(H), 3)
        row: List[float]
        for row in H:
            self.assertEqual(len(row), StateMapping.dimension())
        theta_slice: slice = StateMapping.slice_delta_theta()
        m_slice: slice = StateMapping.slice_delta_m_W()
        i: int
        j: int
        for i in range(3):
            for j in range(StateMapping.dimension()):
                in_theta = theta_slice.start <= j < theta_slice.stop
                in_m = m_slice.start <= j < m_slice.stop
                if not (in_theta or in_m):
                    self.assertEqual(H[i][j], 0.0)

    def test_jacobian_theta_sign(self) -> None:
        """Small angle perturbation matches Jacobian sign."""
        m_W: List[float] = [0.1, -0.2, 0.3]
        state: AhrsState = AhrsState(
            p_WB=[0.0, 0.0, 0.0],
            v_WB=[0.0, 0.0, 0.0],
            q_WB=[1.0, 0.0, 0.0, 0.0],
            omega_WB=[0.0, 0.0, 0.0],
            b_g=[0.0, 0.0, 0.0],
            b_a=[0.0, 0.0, 0.0],
            A_a=_identity3(),
            T_BI=(_identity3(), [0.0, 0.0, 0.0]),
            T_BM=(_identity3(), [0.0, 0.0, 0.0]),
            g_W=[0.0, 0.0, -9.81],
            m_W=m_W,
        )
        H: List[List[float]] = MagModel.jacobian(state)
        theta_slice: slice = StateMapping.slice_delta_theta()
        H_theta: List[List[float]] = [
            H[0][theta_slice.start : theta_slice.stop],
            H[1][theta_slice.start : theta_slice.stop],
            H[2][theta_slice.start : theta_slice.stop],
        ]
        eps: float = 1.0e-6
        delta_theta: List[float] = [eps, 0.0, 0.0]
        delta_x: List[float] = [0.0 for _ in range(StateMapping.dimension())]
        delta_x[theta_slice.start] = eps
        state2: AhrsState = state.apply_error(delta_x)
        pred: List[float] = MagModel.predict(state)
        pred2: List[float] = MagModel.predict(state2)
        delta_pred: List[float] = [
            pred2[0] - pred[0],
            pred2[1] - pred[1],
            pred2[2] - pred[2],
        ]
        delta_lin: List[float] = _matvec3(H_theta, delta_theta)
        i: int
        for i in range(3):
            if abs(delta_lin[i]) > 1.0e-12:
                self.assertEqual(delta_pred[i] > 0.0, delta_lin[i] > 0.0)
            self.assertAlmostEqual(delta_pred[i], delta_lin[i], delta=1.0e-7)


if __name__ == "__main__":
    unittest.main()
