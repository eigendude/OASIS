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
from typing import Callable
from typing import List
from typing import Optional
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
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.timing.replay_engine import ReplayEngine
from oasis_control.localization.ahrs.timing.replay_engine import _EkfProtocol
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


@dataclass(frozen=True, slots=True)
class FakeImuPacket:
    t_meas_ns: int


@dataclass(frozen=True, slots=True)
class FakeMagPacket:
    t_meas_ns: int


@dataclass(frozen=True, slots=True)
class FakeStationaryPacket:
    t_meas_ns: int
    is_stationary: bool


@dataclass(frozen=True, slots=True)
class FakeState:
    t_ns: int


@dataclass(frozen=True, slots=True)
class FakeCovariance:
    t_ns: int


class FakeEkf:
    def __init__(self) -> None:
        self.calls: list[tuple[str, int]] = []
        self._current_time: int = 0

    def propagate_to(self, t_ns: int) -> None:
        self._current_time = t_ns
        self.calls.append(("propagate", t_ns))

    def update_imu(self, imu_packet: FakeImuPacket) -> None:
        self.calls.append(("imu", imu_packet.t_meas_ns))

    def update_mag(self, mag_packet: FakeMagPacket) -> None:
        self.calls.append(("mag", mag_packet.t_meas_ns))

    def update_stationary(self, stationary_packet: FakeStationaryPacket) -> None:
        self.calls.append(("stationary", stationary_packet.t_meas_ns))

    def get_state(self) -> FakeState:
        return FakeState(self._current_time)

    def get_covariance(self) -> FakeCovariance:
        return FakeCovariance(self._current_time)


class TestReplayEngine(unittest.TestCase):
    """Tests for deterministic ReplayEngine behavior."""

    def test_out_of_order_insert_replays(self) -> None:
        """Out-of-order inserts trigger replay in order."""
        ekf: FakeEkf = FakeEkf()
        engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=1000,
            ekf=cast(_EkfProtocol, ekf),
        )
        imu: FakeImuPacket = FakeImuPacket(200)
        mag: FakeMagPacket = FakeMagPacket(150)
        imu_inserted: bool = engine.insert_imu(cast(ImuPacket, imu))
        mag_inserted: bool = engine.insert_mag(cast(MagPacket, mag))
        expected_calls: list[tuple[str, int]] = [
            ("propagate", 200),
            ("imu", 200),
            ("propagate", 150),
            ("mag", 150),
            ("propagate", 200),
            ("imu", 200),
        ]
        self.assertTrue(imu_inserted)
        self.assertTrue(mag_inserted)
        self.assertEqual(ekf.calls, expected_calls)

    def test_duplicate_measurement_rejected(self) -> None:
        """Duplicate measurements do not replay and are diagnosed."""
        ekf: FakeEkf = FakeEkf()
        engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=1000,
            ekf=cast(_EkfProtocol, ekf),
        )
        imu: FakeImuPacket = FakeImuPacket(100)
        first: bool = engine.insert_imu(cast(ImuPacket, imu))
        call_count: int = len(ekf.calls)
        second: bool = engine.insert_imu(cast(ImuPacket, imu))
        self.assertTrue(first)
        self.assertFalse(second)
        self.assertEqual(engine.diagnostics["duplicate_imu"], 1)
        self.assertEqual(len(ekf.calls), call_count)

    def test_publish_frontier_only_on_advance(self) -> None:
        """Publish callback fires only when the frontier advances."""
        ekf: FakeEkf = FakeEkf()
        published: list[int] = []

        def _publish(time_ns: int) -> None:
            published.append(time_ns)

        publish_cb: Callable[[int], None] = _publish
        engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=1000,
            ekf=cast(_EkfProtocol, ekf),
            publish_callback=publish_cb,
        )
        imu: FakeImuPacket = FakeImuPacket(100)
        mag_past: FakeMagPacket = FakeMagPacket(50)
        mag_future: FakeMagPacket = FakeMagPacket(150)
        first: bool = engine.insert_imu(cast(ImuPacket, imu))
        past: bool = engine.insert_mag(cast(MagPacket, mag_past))
        future: bool = engine.insert_mag(cast(MagPacket, mag_future))
        self.assertTrue(first)
        self.assertTrue(past)
        self.assertTrue(future)
        self.assertEqual(published, [100, 150])

    def test_nodes_store_posterior(self) -> None:
        """Nodes store state and covariance after replay."""
        ekf: FakeEkf = FakeEkf()
        engine: ReplayEngine = ReplayEngine(
            t_buffer_ns=1000,
            ekf=cast(_EkfProtocol, ekf),
        )
        imu: FakeImuPacket = FakeImuPacket(10)
        inserted: bool = engine.insert_imu(cast(ImuPacket, imu))
        node: Optional[TimelineNode] = engine.ring_buffer.get(10)
        self.assertTrue(inserted)
        self.assertIsNotNone(node)
        assert node is not None
        self.assertEqual(node.state, FakeState(10))
        self.assertEqual(node.covariance, FakeCovariance(10))


if __name__ == "__main__":
    unittest.main()
