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
from dataclasses import dataclass
from pathlib import Path
from typing import List
from typing import cast


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

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.models.imu_model import ImuModel
from oasis_control.localization.ahrs.models.stationary_detector import (
    StationaryDetector,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState


@dataclass(frozen=True)
class FakeImuPacket:
    """Lightweight packet for testing non-SPD covariance handling."""

    t_meas_ns: int
    z_omega: list[float]
    R_omega: list[list[float]]
    z_accel: list[float]
    R_accel: list[list[float]]


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


class TestStationaryDetector(unittest.TestCase):
    """Tests for stationary detector."""

    def test_deterministic_sorting(self) -> None:
        """Unsorted windows yield deterministic results."""
        state: AhrsState = AhrsState.reset()
        packet1: ImuPacket = ImuPacket(
            t_meas_ns=2,
            frame_id="imu",
            z_omega=[0.0, 0.0, 0.0],
            R_omega=_identity3(),
            z_accel=[0.0, 0.0, 9.81],
            R_accel=_identity3(),
            calibration_prior={},
            calibration_meta={},
        )
        packet2: ImuPacket = ImuPacket(
            t_meas_ns=1,
            frame_id="imu",
            z_omega=[0.0, 0.0, 0.0],
            R_omega=_identity3(),
            z_accel=[0.0, 0.0, 9.81],
            R_accel=_identity3(),
            calibration_prior={},
            calibration_meta={},
        )
        result_unsorted: StationaryPacket = StationaryDetector.detect(
            [packet1, packet2],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=_identity3(),
            R_omega_base=_identity3(),
        )
        result_sorted: StationaryPacket = StationaryDetector.detect(
            [packet2, packet1],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=_identity3(),
            R_omega_base=_identity3(),
        )
        self.assertEqual(result_unsorted.metadata, result_sorted.metadata)
        self.assertEqual(result_unsorted.is_stationary, result_sorted.is_stationary)

    def test_rejects_non_spd_covariance(self) -> None:
        """Non-SPD covariance triggers rejection."""
        state: AhrsState = AhrsState.reset()
        packet: FakeImuPacket = FakeImuPacket(
            t_meas_ns=1,
            z_omega=[0.0, 0.0, 0.0],
            R_omega=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            z_accel=[0.0, 0.0, 9.81],
            R_accel=_identity3(),
        )
        packet_cast: ImuPacket = cast(ImuPacket, packet)
        result: StationaryPacket = StationaryDetector.detect(
            [packet_cast],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=_identity3(),
            R_omega_base=_identity3(),
        )
        self.assertFalse(result.is_stationary)
        self.assertEqual(result.metadata.get("reason"), "R_omega is not SPD")

    def test_stationary_decision(self) -> None:
        """Perfectly stationary measurements pass thresholds."""
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
        z_hat_omega: List[float] = ImuModel.predict_gyro(state)
        z_hat_accel: List[float] = ImuModel.predict_accel(state)
        packet: ImuPacket = ImuPacket(
            t_meas_ns=5,
            frame_id="imu",
            z_omega=z_hat_omega,
            R_omega=_identity3(),
            z_accel=z_hat_accel,
            R_accel=_identity3(),
            calibration_prior={},
            calibration_meta={},
        )
        result: StationaryPacket = StationaryDetector.detect(
            [packet],
            state,
            tau_omega=0.5,
            tau_accel=0.5,
            R_v_base=_identity3(),
            R_omega_base=_identity3(),
        )
        self.assertTrue(result.is_stationary)


if __name__ == "__main__":
    unittest.main()
