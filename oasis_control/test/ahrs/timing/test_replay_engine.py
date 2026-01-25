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
from typing import Callable
from typing import List
from typing import Mapping


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
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping
from oasis_control.localization.ahrs.timing.replay_engine import ReplayEngine
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


def _identity(size: int) -> List[List[float]]:
    matrix: List[List[float]] = []
    row: int
    for row in range(size):
        values: List[float] = [0.0 for _ in range(size)]
        values[row] = 1.0
        matrix.append(values)
    return matrix


def _make_imu_packet(t_meas_ns: int) -> ImuPacket:
    R: List[List[float]] = _identity(3)
    calibration_prior: Mapping[str, object] = {"bias": [0.0, 0.0, 0.0]}
    calibration_meta: Mapping[str, object] = {"source": "test"}
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
        z_m=[0.1, 0.2, 0.3],
        R_m_raw=R,
    )


def _make_stationary_packet(t_meas_ns: int, is_stationary: bool) -> StationaryPacket:
    R: List[List[float]] = _identity(3)
    metadata: Mapping[str, object] = {"score": 0.2}
    return StationaryPacket(
        t_meas_ns=t_meas_ns,
        window_start_ns=t_meas_ns - 5,
        window_end_ns=t_meas_ns,
        is_stationary=is_stationary,
        R_v=R,
        R_omega=None,
        metadata=metadata,
    )


def _state_for_time(t_ns: int) -> AhrsState:
    base: AhrsState = AhrsState.reset()
    return AhrsState(
        p_WB=[float(t_ns), base.p_WB[1], base.p_WB[2]],
        v_WB=base.v_WB,
        q_WB=base.q_WB,
        omega_WB=base.omega_WB,
        b_g=base.b_g,
        b_a=base.b_a,
        A_a=base.A_a,
        T_BI=base.T_BI,
        T_BM=base.T_BM,
        g_W=base.g_W,
        m_W=base.m_W,
    )


def _covariance_for_time(t_ns: int) -> AhrsCovariance:
    size: int = StateMapping.dimension()
    matrix: List[List[float]] = _identity(size)
    matrix[0][0] = 1.0 + float(t_ns)
    return AhrsCovariance.from_matrix(matrix)


class FakeEkf:
    """Minimal EKF stub for replay tests."""

    def __init__(self) -> None:
        self.calls: List[str] = []
        self.last_time_ns: int = 0
        self._calibration_prior_applied: bool = False

    def propagate_to(self, t_ns: int) -> None:
        self.last_time_ns = t_ns
        self.calls.append(f"prop:{t_ns}")

    def update_imu(self, imu_packet: ImuPacket) -> None:
        self.calls.append(f"imu:{imu_packet.t_meas_ns}")

    def update_mag(self, mag_packet: MagPacket) -> None:
        self.calls.append(f"mag:{mag_packet.t_meas_ns}")

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        self.calls.append(f"stationary:{stationary_packet.t_meas_ns}")

    def get_state(self) -> AhrsState:
        return _state_for_time(self.last_time_ns)

    def get_covariance(self) -> AhrsCovariance:
        return _covariance_for_time(self.last_time_ns)

    def restore(
        self,
        *,
        t_ns: int,
        state: AhrsState,
        covariance: AhrsCovariance,
        calibration_prior_applied: bool,
    ) -> None:
        _ = state
        _ = covariance
        self.last_time_ns = t_ns
        self._calibration_prior_applied = calibration_prior_applied
        self.calls.append(f"restore:{t_ns}")

    def get_calibration_prior_applied(self) -> bool:
        return self._calibration_prior_applied


class FakeEkfMonotonic:
    """EKF stub that enforces monotonic propagation and records restores."""

    def __init__(self) -> None:
        self.calls: List[str] = []
        self.restore_calls: List[int] = []
        self.last_time_ns: int = 0
        self._calibration_prior_applied: bool = False
        self._state: AhrsState = AhrsState.reset()
        self._covariance: AhrsCovariance = AhrsCovariance.from_matrix(
            _identity(StateMapping.dimension())
        )

    def propagate_to(self, t_ns: int) -> None:
        if t_ns < self.last_time_ns:
            raise ValueError("timestamps must be monotonic")
        self.last_time_ns = t_ns
        self.calls.append(f"prop:{t_ns}")

    def update_imu(self, imu_packet: ImuPacket) -> None:
        self.calls.append(f"imu:{imu_packet.t_meas_ns}")

    def update_mag(self, mag_packet: MagPacket) -> None:
        self.calls.append(f"mag:{mag_packet.t_meas_ns}")

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        self.calls.append(f"stationary:{stationary_packet.t_meas_ns}")

    def get_state(self) -> AhrsState:
        return _state_for_time(self.last_time_ns)

    def get_covariance(self) -> AhrsCovariance:
        return _covariance_for_time(self.last_time_ns)

    def restore(
        self,
        *,
        t_ns: int,
        state: AhrsState,
        covariance: AhrsCovariance,
        calibration_prior_applied: bool,
    ) -> None:
        self.last_time_ns = t_ns
        self._state = state
        self._covariance = covariance
        self._calibration_prior_applied = calibration_prior_applied
        self.restore_calls.append(t_ns)
        self.calls.append(f"restore:{t_ns}")

    def get_calibration_prior_applied(self) -> bool:
        return self._calibration_prior_applied


class FakeEkfAliasing:
    """EKF stub that returns mutable internals for aliasing tests."""

    def __init__(self) -> None:
        self.calls: List[str] = []
        self.last_time_ns: int = 0
        self._calibration_prior_applied: bool = False
        self._state: AhrsState = AhrsState.reset()
        size: int = StateMapping.dimension()
        self._covariance_matrix: List[List[float]] = _identity(size)
        self._covariance: AhrsCovariance = AhrsCovariance(P=self._covariance_matrix)

    def propagate_to(self, t_ns: int) -> None:
        self.last_time_ns = t_ns
        self._state.p_WB[0] = float(t_ns)
        self._covariance_matrix[0][0] = float(t_ns)
        self.calls.append(f"prop:{t_ns}")

    def update_imu(self, imu_packet: ImuPacket) -> None:
        self.calls.append(f"imu:{imu_packet.t_meas_ns}")

    def update_mag(self, mag_packet: MagPacket) -> None:
        self.calls.append(f"mag:{mag_packet.t_meas_ns}")

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        self.calls.append(f"stationary:{stationary_packet.t_meas_ns}")

    def get_state(self) -> AhrsState:
        return self._state

    def get_covariance(self) -> AhrsCovariance:
        return self._covariance

    def restore(
        self,
        *,
        t_ns: int,
        state: AhrsState,
        covariance: AhrsCovariance,
        calibration_prior_applied: bool,
    ) -> None:
        _ = state
        _ = covariance
        self.last_time_ns = t_ns
        self._state.p_WB[0] = float(t_ns)
        self._covariance_matrix[0][0] = float(t_ns)
        self._calibration_prior_applied = calibration_prior_applied
        self.calls.append(f"restore:{t_ns}")

    def get_calibration_prior_applied(self) -> bool:
        return self._calibration_prior_applied


class TestReplayEngine(unittest.TestCase):
    """Tests for ReplayEngine."""

    def test_out_of_order_replay_and_publish(self) -> None:
        """Out-of-order inserts replay deterministically and publish once."""
        ekf: FakeEkf = FakeEkf()
        published: List[int] = []

        def publish_callback(t_ns: int) -> None:
            published.append(t_ns)

        engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=1_000,
            ekf=ekf,
            publish_callback=publish_callback,
        )
        imu_200: ImuPacket = _make_imu_packet(200)
        mag_100: MagPacket = _make_mag_packet(100)
        self.assertTrue(engine.insert_imu(imu_200))
        self.assertTrue(engine.insert_mag(mag_100))
        self.assertEqual(
            ekf.calls,
            [
                "prop:200",
                "imu:200",
                "restore:0",
                "prop:100",
                "mag:100",
                "prop:200",
                "imu:200",
            ],
        )
        self.assertEqual(published, [200])
        node_100: TimelineNode | None = engine.ring_buffer.get(100)
        node_200: TimelineNode | None = engine.ring_buffer.get(200)
        self.assertIsNotNone(node_100)
        self.assertIsNotNone(node_200)
        if node_100 is not None and node_200 is not None:
            self.assertIsNotNone(node_100.state)
            self.assertIsNotNone(node_100.covariance)
            self.assertIsNotNone(node_200.state)
            if node_100.state is not None and node_200.state is not None:
                self.assertEqual(node_100.state.p_WB[0], 100.0)
                self.assertEqual(node_200.state.p_WB[0], 200.0)

    def test_duplicate_measurement_rejected(self) -> None:
        """Duplicate measurements are rejected with diagnostics."""
        ekf: FakeEkf = FakeEkf()
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=500, ekf=ekf)
        imu_100: ImuPacket = _make_imu_packet(100)
        self.assertTrue(engine.insert_imu(imu_100))
        self.assertFalse(engine.insert_imu(imu_100))
        self.assertEqual(engine.diagnostics["duplicate_imu"], 1)

    def test_stationary_updates_only_when_true(self) -> None:
        """Stationary updates are applied only when stationary is true."""
        ekf: FakeEkf = FakeEkf()
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=500, ekf=ekf)
        stationary_false: StationaryPacket = _make_stationary_packet(100, False)
        stationary_true: StationaryPacket = _make_stationary_packet(200, True)
        self.assertTrue(engine.insert_stationary(stationary_false))
        self.assertTrue(engine.insert_stationary(stationary_true))
        self.assertIn("stationary:200", ekf.calls)
        self.assertNotIn("stationary:100", ekf.calls)

    def test_publish_only_when_frontier_advances(self) -> None:
        """Publish callback fires only when frontier advances."""
        ekf: FakeEkf = FakeEkf()
        published: List[int] = []
        callback: Callable[[int], None] = published.append
        engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=1_000,
            ekf=ekf,
            publish_callback=callback,
        )
        imu_300: ImuPacket = _make_imu_packet(300)
        imu_200: ImuPacket = _make_imu_packet(200)
        self.assertTrue(engine.insert_imu(imu_300))
        self.assertTrue(engine.insert_imu(imu_200))
        self.assertEqual(published, [300])

    def test_out_of_order_insert_does_not_raise_and_replays_forward(self) -> None:
        """Out-of-order inserts restore and replay forward without errors."""
        ekf: FakeEkfMonotonic = FakeEkfMonotonic()
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=1_000, ekf=ekf)
        imu_200: ImuPacket = _make_imu_packet(200)
        mag_100: MagPacket = _make_mag_packet(100)
        self.assertTrue(engine.insert_imu(imu_200))
        first_calls: int = len(ekf.calls)
        self.assertTrue(engine.insert_mag(mag_100))
        new_calls: List[str] = ekf.calls[first_calls:]

        restore_index: int = new_calls.index("restore:0")
        restore_time: int = int(new_calls[restore_index].split(":")[1])
        prop_times: List[int] = []
        call: str
        for call in new_calls[restore_index + 1 :]:
            if call.startswith("prop:"):
                prop_times.append(int(call.split(":")[1]))
        self.assertFalse(
            any(call.startswith("prop:") for call in new_calls[:restore_index])
        )
        self.assertTrue(prop_times)
        self.assertGreaterEqual(prop_times[0], restore_time)
        self.assertEqual(prop_times, sorted(prop_times))
        self.assertEqual(engine.frontier_time(), 200)

        node_100: TimelineNode | None = engine.ring_buffer.get(100)
        node_200: TimelineNode | None = engine.ring_buffer.get(200)
        self.assertIsNotNone(node_100)
        self.assertIsNotNone(node_200)
        if node_100 is not None and node_200 is not None:
            self.assertIsNotNone(node_100.state)
            self.assertIsNotNone(node_100.covariance)
            self.assertIsNotNone(node_200.state)
            self.assertIsNotNone(node_200.covariance)

    def test_restore_uses_previous_complete_node_when_available(self) -> None:
        """Restore uses the latest complete node before the replay window."""
        ekf: FakeEkfMonotonic = FakeEkfMonotonic()
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=1_000, ekf=ekf)
        imu_100: ImuPacket = _make_imu_packet(100)
        imu_200: ImuPacket = _make_imu_packet(200)
        mag_150: MagPacket = _make_mag_packet(150)

        self.assertTrue(engine.insert_imu(imu_100))
        self.assertTrue(engine.insert_imu(imu_200))
        self.assertTrue(engine.insert_mag(mag_150))

        self.assertIn(100, ekf.restore_calls)
        restore_index: int = ekf.calls.index("restore:100")
        replay_props: List[int] = []
        call: str
        for call in ekf.calls[restore_index + 1 :]:
            if call.startswith("prop:"):
                replay_props.append(int(call.split(":")[1]))
        self.assertEqual(replay_props[0], 150)
        self.assertEqual(replay_props[-1], 200)

    def test_restore_falls_back_to_initial_when_no_complete_prior(self) -> None:
        """Restore falls back to the initial snapshot when needed."""
        ekf: FakeEkfMonotonic = FakeEkfMonotonic()
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=1_000, ekf=ekf)
        imu_200: ImuPacket = _make_imu_packet(200)
        mag_100: MagPacket = _make_mag_packet(100)

        self.assertTrue(engine.insert_imu(imu_200))
        self.assertTrue(engine.insert_mag(mag_100))

        self.assertIn(0, ekf.restore_calls)
        restore_index: int = ekf.calls.index("restore:0")
        replay_props: List[int] = []
        call: str
        for call in ekf.calls[restore_index + 1 :]:
            if call.startswith("prop:"):
                replay_props.append(int(call.split(":")[1]))
        self.assertEqual(replay_props, [100, 200])

    def test_snapshots_do_not_alias_ekf_internals(self) -> None:
        """Replay snapshots do not alias EKF internal state or covariance."""
        ekf: FakeEkfAliasing = FakeEkfAliasing()
        engine: ReplayEngine = ReplayEngine(t_buffer_ns=1_000, ekf=ekf)
        imu_100: ImuPacket = _make_imu_packet(100)
        imu_200: ImuPacket = _make_imu_packet(200)

        self.assertTrue(engine.insert_imu(imu_100))
        node_100: TimelineNode | None = engine.ring_buffer.get(100)
        self.assertIsNotNone(node_100)
        if node_100 is None:
            self.fail("Expected node at 100 ns")
        self.assertIsNotNone(node_100.state)
        self.assertIsNotNone(node_100.covariance)
        if node_100.state is None or node_100.covariance is None:
            self.fail("Expected snapshot data at 100 ns")
        state_snapshot: float = node_100.state.p_WB[0]
        covariance_snapshot: float = node_100.covariance.as_matrix()[0][0]

        self.assertTrue(engine.insert_imu(imu_200))
        self.assertEqual(node_100.state.p_WB[0], state_snapshot)
        self.assertEqual(node_100.covariance.as_matrix()[0][0], covariance_snapshot)
