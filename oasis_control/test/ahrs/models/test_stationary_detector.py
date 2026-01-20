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

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.models.stationary_detector import (
    StationaryDetector,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState


def _identity3() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _make_packet(t_meas_ns: int) -> ImuPacket:
    return ImuPacket(
        t_meas_ns=t_meas_ns,
        frame_id="imu",
        z_omega=[0.0, 0.0, 0.0],
        R_omega=_identity3(),
        z_accel=[0.0, 0.0, 9.81],
        R_accel=_identity3(),
        calibration_prior={},
        calibration_meta={},
    )


class TestStationaryDetector(unittest.TestCase):
    """Tests for the StationaryDetector behavior."""

    def test_sorting_determinism(self) -> None:
        """Unsorted windows produce deterministic results."""
        state: AhrsState = AhrsState.reset()
        packet1: ImuPacket = _make_packet(20)
        packet2: ImuPacket = _make_packet(10)
        R_v_base: List[List[float]] = _identity3()
        R_omega_base: List[List[float]] = _identity3()
        result_unsorted: StationaryPacket = StationaryDetector.detect(
            [packet1, packet2],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=R_v_base,
            R_omega_base=R_omega_base,
        )
        result_sorted: StationaryPacket = StationaryDetector.detect(
            [packet2, packet1],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=R_v_base,
            R_omega_base=R_omega_base,
        )
        self.assertEqual(result_unsorted.is_stationary, result_sorted.is_stationary)
        self.assertEqual(result_unsorted.metadata, result_sorted.metadata)

    def test_rejects_non_spd_covariance(self) -> None:
        """Non-SPD covariances reject the window."""
        state: AhrsState = AhrsState.reset()
        packet: ImuPacket = _make_packet(10)
        non_spd: List[List[float]] = [
            [1.0, 2.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        object.__setattr__(packet, "R_omega", non_spd)
        result: StationaryPacket = StationaryDetector.detect(
            [packet],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=_identity3(),
            R_omega_base=_identity3(),
        )
        self.assertFalse(result.is_stationary)
        self.assertEqual(result.metadata.get("reason"), "non-spd covariance in window")

    def test_stationary_for_perfect_measurements(self) -> None:
        """Perfect zero-motion measurements yield a stationary decision."""
        state: AhrsState = AhrsState.reset()
        packet: ImuPacket = _make_packet(100)
        result: StationaryPacket = StationaryDetector.detect(
            [packet],
            state,
            tau_omega=1.0,
            tau_accel=1.0,
            R_v_base=_identity3(),
            R_omega_base=_identity3(),
        )
        self.assertTrue(result.is_stationary)


if __name__ == "__main__":
    unittest.main()
