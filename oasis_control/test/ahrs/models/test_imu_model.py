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

from oasis_control.localization.ahrs.models.imu_model import ImuModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


def _identity3() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


class TestImuModel(unittest.TestCase):
    """Tests for the ImuModel predictions and Jacobians."""

    def test_predict_accel_rest(self) -> None:
        """At rest, accel prediction matches -g in {I}."""
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
        prediction: List[float] = ImuModel.predict_accel(state)
        self.assertEqual(prediction, [0.0, 0.0, 9.81])

    def test_residual_sign(self) -> None:
        """Residuals use ν = z - z_hat."""
        z_omega: List[float] = [1.0, 2.0, 3.0]
        z_hat_omega: List[float] = [0.5, 1.5, 2.5]
        nu_omega: List[float] = ImuModel.residual_gyro(z_omega, z_hat_omega)
        self.assertEqual(nu_omega, [0.5, 0.5, 0.5])
        z_accel: List[float] = [4.0, 5.0, 6.0]
        z_hat_accel: List[float] = [1.0, 2.0, 3.0]
        nu_accel: List[float] = ImuModel.residual_accel(z_accel, z_hat_accel)
        self.assertEqual(nu_accel, [3.0, 3.0, 3.0])

    def test_jacobian_blocks(self) -> None:
        """Jacobians populate only the expected error-state blocks."""
        state: AhrsState = AhrsState.reset()
        H_gyro: List[List[float]] = ImuModel.jacobian_gyro(state)
        self.assertEqual(len(H_gyro), 3)
        self.assertEqual(len(H_gyro[0]), StateMapping.dimension())
        delta_omega: slice = StateMapping.slice_delta_omega()
        delta_b_g: slice = StateMapping.slice_delta_b_g()
        allowed_gyro: set[int] = set(range(delta_omega.start, delta_omega.stop))
        allowed_gyro |= set(range(delta_b_g.start, delta_b_g.stop))
        for i in range(3):
            for j in range(StateMapping.dimension()):
                if j not in allowed_gyro:
                    self.assertEqual(H_gyro[i][j], 0.0)

        H_accel: List[List[float]] = ImuModel.jacobian_accel(state)
        self.assertEqual(len(H_accel), 3)
        self.assertEqual(len(H_accel[0]), StateMapping.dimension())
        delta_theta: slice = StateMapping.slice_delta_theta()
        delta_A_a: slice = StateMapping.slice_delta_A_a()
        delta_b_a: slice = StateMapping.slice_delta_b_a()
        delta_g_W: slice = StateMapping.slice_delta_g_W()
        allowed_accel: set[int] = set(range(delta_theta.start, delta_theta.stop))
        allowed_accel |= set(range(delta_A_a.start, delta_A_a.stop))
        allowed_accel |= set(range(delta_b_a.start, delta_b_a.stop))
        allowed_accel |= set(range(delta_g_W.start, delta_g_W.stop))
        for i in range(3):
            for j in range(StateMapping.dimension()):
                if j not in allowed_accel:
                    self.assertEqual(H_accel[i][j], 0.0)


if __name__ == "__main__":
    unittest.main()
