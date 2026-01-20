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
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


class TestImuModel(unittest.TestCase):
    """Tests for IMU model."""

    def test_predict_accel_rest(self) -> None:
        """At rest, predicted accel returns +g in {I}."""
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
        accel: List[float] = ImuModel.predict_accel(state)
        self.assertEqual(accel, [0.0, 0.0, 9.81])

    def test_residual_sign(self) -> None:
        """Residuals follow ν = z - z_hat."""
        z_omega: List[float] = [1.0, 2.0, 3.0]
        z_hat: List[float] = [0.5, 1.5, 2.5]
        self.assertEqual(ImuModel.residual_gyro(z_omega, z_hat), [0.5, 0.5, 0.5])
        z_accel: List[float] = [4.0, 5.0, 6.0]
        z_hat_accel: List[float] = [1.0, 2.0, 3.0]
        self.assertEqual(ImuModel.residual_accel(z_accel, z_hat_accel), [3.0, 3.0, 3.0])

    def test_jacobian_shapes(self) -> None:
        """Jacobians have correct shape and sparsity."""
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
        H_gyro: List[List[float]] = ImuModel.jacobian_gyro(state)
        self.assertEqual(len(H_gyro), 3)
        row: List[float]
        for row in H_gyro:
            self.assertEqual(len(row), StateMapping.dimension())
        omega_slice: slice = StateMapping.slice_delta_omega()
        b_g_slice: slice = StateMapping.slice_delta_b_g()
        i: int
        j: int
        for i in range(3):
            for j in range(StateMapping.dimension()):
                in_omega = omega_slice.start <= j < omega_slice.stop
                in_b_g = b_g_slice.start <= j < b_g_slice.stop
                if not (in_omega or in_b_g):
                    self.assertEqual(H_gyro[i][j], 0.0)
        H_accel: List[List[float]] = ImuModel.jacobian_accel(state)
        self.assertEqual(len(H_accel), 3)
        row_accel: List[float] = []
        for row_accel in H_accel:
            self.assertEqual(len(row_accel), StateMapping.dimension())
        theta_slice: slice = StateMapping.slice_delta_theta()
        A_slice: slice = StateMapping.slice_delta_A_a()
        b_a_slice: slice = StateMapping.slice_delta_b_a()
        g_slice: slice = StateMapping.slice_delta_g_W()
        for i in range(3):
            for j in range(StateMapping.dimension()):
                in_theta = theta_slice.start <= j < theta_slice.stop
                in_A = A_slice.start <= j < A_slice.stop
                in_b_a = b_a_slice.start <= j < b_a_slice.stop
                in_g = g_slice.start <= j < g_slice.stop
                if not (in_theta or in_A or in_b_a or in_g):
                    self.assertEqual(H_accel[i][j], 0.0)


if __name__ == "__main__":
    unittest.main()
