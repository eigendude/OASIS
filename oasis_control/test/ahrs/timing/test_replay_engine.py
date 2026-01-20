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

from dataclasses import dataclass
from dataclasses import replace
from typing import cast

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


@dataclass(frozen=True, slots=True)
class FakePacket:
    t_meas_ns: int
    is_stationary: bool = True


class FakeEkf:
    def __init__(self) -> None:
        self.events: list[tuple[str, int]] = []
        self.current_time: int = 0
        self._state: AhrsState = _state_for_time(0)
        self._covariance: AhrsCovariance = _covariance_for_time(0)

    def propagate_to(self, t_ns: int) -> None:
        self.current_time = t_ns
        self.events.append(("propagate", t_ns))
        self._state = _state_for_time(t_ns)
        self._covariance = _covariance_for_time(t_ns)

    def update_imu(self, imu_packet: ImuPacket) -> None:
        self.events.append(("imu", imu_packet.t_meas_ns))

    def update_mag(self, mag_packet: MagPacket) -> None:
        self.events.append(("mag", mag_packet.t_meas_ns))

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        self.events.append(("stationary", stationary_packet.t_meas_ns))

    def get_state(self) -> AhrsState:
        return self._state

    def get_covariance(self) -> AhrsCovariance:
        return self._covariance


def _state_for_time(t_ns: int) -> AhrsState:
    base: AhrsState = AhrsState.reset()
    return replace(base, p_WB=[float(t_ns), 0.0, 0.0])


def _covariance_for_time(t_ns: int) -> AhrsCovariance:
    size: int = StateMapping.dimension()
    matrix: list[list[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    for i in range(size):
        matrix[i][i] = 1.0
    matrix[0][0] = float(t_ns)
    return AhrsCovariance.from_matrix(matrix)


def test_out_of_order_insert_triggers_replay() -> None:
    ekf: FakeEkf = FakeEkf()
    engine: ReplayEngine = ReplayEngine(t_buffer_ns=100, ekf=ekf)

    imu_packet_new: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=20))
    imu_packet_old: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=10))

    assert engine.insert_imu(imu_packet_new)
    assert engine.insert_imu(imu_packet_old)

    assert ekf.events == [
        ("propagate", 20),
        ("imu", 20),
        ("propagate", 10),
        ("imu", 10),
        ("propagate", 20),
        ("imu", 20),
    ]


def test_duplicate_measurement_rejected() -> None:
    ekf: FakeEkf = FakeEkf()
    engine: ReplayEngine = ReplayEngine(t_buffer_ns=50, ekf=ekf)
    imu_packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=5))

    assert engine.insert_imu(imu_packet)
    assert not engine.insert_imu(imu_packet)
    assert engine.diagnostics["duplicate_imu"] == 1


def test_publish_callback_only_on_frontier_advances() -> None:
    ekf: FakeEkf = FakeEkf()
    published: list[int] = []

    def _publish(frontier: int) -> None:
        published.append(frontier)

    engine: ReplayEngine = ReplayEngine(
        t_buffer_ns=100,
        ekf=ekf,
        publish_callback=_publish,
    )
    first_packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=10))
    old_packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=5))
    new_packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=20))

    assert engine.insert_imu(first_packet)
    assert engine.insert_imu(old_packet)
    assert engine.insert_imu(new_packet)

    assert published == [10, 20]


def test_nodes_store_posterior_state_and_covariance() -> None:
    ekf: FakeEkf = FakeEkf()
    engine: ReplayEngine = ReplayEngine(t_buffer_ns=100, ekf=ekf)
    imu_packet: ImuPacket = cast(ImuPacket, FakePacket(t_meas_ns=7))
    mag_packet: MagPacket = cast(MagPacket, FakePacket(t_meas_ns=7))

    assert engine.insert_imu(imu_packet)
    assert engine.insert_mag(mag_packet)

    node: TimelineNode | None = engine.ring_buffer.get(7)
    assert node is not None
    assert node.state == _state_for_time(7)
    assert node.covariance == _covariance_for_time(7)
