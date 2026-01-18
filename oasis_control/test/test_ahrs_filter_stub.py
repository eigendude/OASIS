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

from builtin_interfaces.msg import Time as TimeMsg

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_filter import AhrsFilter
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import MagSample


class FixedClock(AhrsClock):
    def __init__(self, now_sec: int, now_nanosec: int) -> None:
        self._now_sec: int = now_sec
        self._now_nanosec: int = now_nanosec

    def now_ros_time(self) -> TimeMsg:
        stamp: TimeMsg = TimeMsg()
        stamp.sec = self._now_sec
        stamp.nanosec = self._now_nanosec

        return stamp


def _config() -> AhrsConfig:
    return AhrsConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=0.5,
        epsilon_wall_future_sec=1.0,
        dt_clock_jump_max_sec=0.0,
        dt_imu_max_sec=0.0,
        mag_alpha=1.0,
        mag_r_min=[0.0] * 9,
        mag_r_max=[0.0] * 9,
        mag_r0=[0.0] * 9,
        q_v=0.0,
        q_w=0.0,
        q_bg=0.0,
        q_ba=0.0,
        q_a=0.0,
        q_bi=0.0,
        q_bm=0.0,
        q_g=0.0,
        q_m=0.0,
    )


def _event_at(sec: int, nanosec: int) -> AhrsEvent:
    sample: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=[0.0] * 9,
    )

    return AhrsEvent(
        t_meas=AhrsTime(sec=sec, nanosec=nanosec),
        topic="magnetic_field",
        frame_id="mag",
        event_type=AhrsEventType.MAG,
        payload=sample,
    )


def test_filter_frontier_advances() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(), clock=clock)

    first = filt.handle_event(_event_at(1, 0))
    second = filt.handle_event(_event_at(2, 0))

    assert first.frontier_advanced is True
    assert second.frontier_advanced is True
    assert first.state is not None
    assert second.state is not None
    assert first.state.state_seq == 1
    assert second.state.state_seq == 2


def test_filter_out_of_order_does_not_advance() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(), clock=clock)

    first = filt.handle_event(_event_at(2, 0))
    out_of_order = filt.handle_event(_event_at(1, 0))
    third = filt.handle_event(_event_at(3, 0))

    assert first.frontier_advanced is True
    assert out_of_order.frontier_advanced is False
    assert out_of_order.state is None
    assert third.state is not None
    assert third.state.state_seq == 2
    assert third.diagnostics is not None
    assert third.diagnostics.diag_seq == 2
