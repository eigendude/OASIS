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
from typing import Mapping
from typing import Sequence


ROOT: Path = Path(__file__).resolve().parents[3]
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
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.filter.ekf import AhrsEkf
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.replay_engine import ReplayEngine
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


def _identity(size: int) -> List[List[float]]:
    matrix: List[List[float]] = []
    row_idx: int
    for row_idx in range(size):
        row: List[float] = [0.0 for _ in range(size)]
        row[row_idx] = 1.0
        matrix.append(row)
    return matrix


def _zero_matrix(size: int) -> List[List[float]]:
    matrix: List[List[float]] = []
    row_idx: int
    for row_idx in range(size):
        matrix.append([0.0 for _ in range(size)])
    return matrix


def _make_imu_packet(t_meas_ns: int) -> ImuPacket:
    R: List[List[float]] = _identity(3)
    calibration_prior: Mapping[str, object] = {"bias": [0.0, 0.0, 0.0]}
    calibration_meta: Mapping[str, object] = {"source": "integration"}
    return ImuPacket(
        t_meas_ns=t_meas_ns,
        frame_id="imu",
        z_omega=[0.1, 0.2, 0.3],
        R_omega=R,
        z_accel=[0.0, 0.0, 9.81],
        R_accel=R,
        calibration_prior=calibration_prior,
        calibration_meta=calibration_meta,
    )


def _make_mag_packet(t_meas_ns: int) -> MagPacket:
    R: List[List[float]] = _identity(3)
    return MagPacket(
        t_meas_ns=t_meas_ns,
        frame_id="mag",
        z_m=[0.1, 0.0, 0.3],
        R_m_raw=R,
    )


def _make_stationary_packet(t_meas_ns: int, is_stationary: bool) -> StationaryPacket:
    R: List[List[float]] = _identity(3)
    metadata: Mapping[str, object] = {"score": 0.75}
    return StationaryPacket(
        t_meas_ns=t_meas_ns,
        window_start_ns=t_meas_ns - 10,
        window_end_ns=t_meas_ns,
        is_stationary=is_stationary,
        R_v=R,
        R_omega=R,
        metadata=metadata,
    )


def _is_symmetric(matrix: Sequence[Sequence[float]], tol: float) -> bool:
    size: int = len(matrix)
    i: int
    for i in range(size):
        j: int
        for j in range(i + 1, size):
            if abs(matrix[i][j] - matrix[j][i]) > tol:
                return False
    return True


class EkfAdapter:
    """Adapter to align AhrsEkf with ReplayEngine's EKF protocol."""

    def __init__(self, ekf: AhrsEkf) -> None:
        self._ekf: AhrsEkf = ekf
        self._last_t_ns: int = 0

    def propagate_to(self, t_ns: int) -> None:
        dt_ns: int = t_ns - self._last_t_ns
        if dt_ns < 0:
            dt_ns = 0
        if dt_ns > 0:
            self._ekf.propagate_to(t_ns)
        self._last_t_ns = t_ns

    def update_imu(self, imu_packet: ImuPacket) -> None:
        self._ekf.update_imu(imu_packet)

    def update_mag(self, mag_packet: MagPacket) -> None:
        self._ekf.update_mag(mag_packet)

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        if not stationary_packet.is_stationary:
            return
        self._ekf.update_stationary(stationary_packet)

    def get_state(self) -> AhrsState:
        return self._ekf.get_state()

    def get_covariance(self) -> AhrsCovariance:
        return self._ekf.get_covariance()


class TestAhrsReplayPipeline(unittest.TestCase):
    """Integration test for ReplayEngine with AhrsEkf."""

    def test_replay_pipeline_with_out_of_order_packets(self) -> None:
        Q_c: List[List[float]] = _zero_matrix(39)
        ekf: AhrsEkf = AhrsEkf(Q_c=Q_c)
        adapter: EkfAdapter = EkfAdapter(ekf)
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=1_000, ekf=adapter)

        imu_200: ImuPacket = _make_imu_packet(200)
        mag_200: MagPacket = _make_mag_packet(200)
        stationary_100_false: StationaryPacket = _make_stationary_packet(100, False)
        imu_100: ImuPacket = _make_imu_packet(100)
        stationary_200_true: StationaryPacket = _make_stationary_packet(200, True)

        self.assertTrue(engine.insert_imu(imu_200))
        self.assertTrue(engine.insert_mag(mag_200))
        self.assertTrue(engine.insert_stationary(stationary_100_false))

        self.assertNotIn("no_turn", ekf.last_reports)
        self.assertNotIn("zupt", ekf.last_reports)

        self.assertTrue(engine.insert_imu(imu_100))
        self.assertTrue(engine.insert_stationary(stationary_200_true))

        self.assertEqual(engine.frontier_time(), 200)

        node_100: TimelineNode | None = engine.ring_buffer.get(100)
        node_200: TimelineNode | None = engine.ring_buffer.get(200)
        self.assertIsNotNone(node_100)
        self.assertIsNotNone(node_200)

        if node_100 is not None:
            self.assertTrue(node_100.is_complete())
            self.assertIsInstance(node_100.state, AhrsState)
            self.assertIsInstance(node_100.covariance, AhrsCovariance)
        if node_200 is not None:
            self.assertTrue(node_200.is_complete())
            self.assertIsInstance(node_200.state, AhrsState)
            self.assertIsInstance(node_200.covariance, AhrsCovariance)

        if node_200 is not None and node_200.covariance is not None:
            covariance: AhrsCovariance = node_200.covariance
            matrix: List[List[float]] = covariance.as_matrix()
            self.assertEqual(len(matrix), StateMapping.dimension())
            self.assertTrue(_is_symmetric(matrix, tol=1e-9))

        self.assertEqual(engine.diagnostics["duplicate_imu"], 0)
        self.assertEqual(engine.diagnostics["duplicate_mag"], 0)
        self.assertEqual(engine.diagnostics["duplicate_stationary"], 0)
        self.assertEqual(engine.diagnostics["reject_too_old"], 0)


if __name__ == "__main__":
    unittest.main()
