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

import pytest

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_filter import AhrsFilter
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsImuPacket
from oasis_control.localization.ahrs.ahrs_types import AhrsOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import ImuCalibrationData
from oasis_control.localization.ahrs.ahrs_types import ImuSample
from oasis_control.localization.ahrs.ahrs_types import MagSample


class FixedClock(AhrsClock):
    def __init__(self, now_sec: int, now_nanosec: int) -> None:
        self._now_sec: int = now_sec
        self._now_nanosec: int = now_nanosec

    def now(self) -> AhrsTime:
        return AhrsTime(sec=self._now_sec, nanosec=self._now_nanosec)


def _config(
    *,
    t_buffer_sec: float = 0.5,
    dt_clock_jump_max_sec: float = 0.0,
    dt_imu_max_sec: float = 0.0,
) -> AhrsConfig:
    return AhrsConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=t_buffer_sec,
        epsilon_wall_future_sec=1.0,
        dt_clock_jump_max_sec=dt_clock_jump_max_sec,
        dt_imu_max_sec=dt_imu_max_sec,
        gyro_gate_d2_threshold=9.0,
        accel_gate_d2_threshold=9.0,
        accel_use_direction_only=False,
        mag_gate_d2_threshold=9.0,
        mag_use_direction_only=True,
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


def _mag_event_at(sec: int, nanosec: int, field_t: list[float]) -> AhrsEvent:
    sample: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=list(field_t),
        magnetic_field_cov=[0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
    )

    return AhrsEvent(
        t_meas=AhrsTime(sec=sec, nanosec=nanosec),
        topic="magnetic_field",
        frame_id="mag",
        event_type=AhrsEventType.MAG,
        payload=sample,
    )


def _imu_event_at(sec: int, nanosec: int) -> AhrsEvent:
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, -9.81],
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=[0.0] * 9,
    )
    calibration: ImuCalibrationData = ImuCalibrationData(
        valid=True,
        frame_id="imu",
        accel_bias_mps2=[0.0, 0.0, 0.0],
        accel_a=[0.0] * 9,
        accel_param_cov=[0.0] * 144,
        gyro_bias_rps=[0.0, 0.0, 0.0],
        gyro_bias_cov=[0.0] * 9,
        gravity_mps2=9.81,
        fit_sample_count=0,
        rms_residual_mps2=0.0,
        temperature_c=20.0,
        temperature_var_c2=0.0,
    )
    packet: AhrsImuPacket = AhrsImuPacket(imu=imu, calibration=calibration)

    return AhrsEvent(
        t_meas=AhrsTime(sec=sec, nanosec=nanosec),
        topic="imu",
        frame_id="imu",
        event_type=AhrsEventType.IMU,
        payload=packet,
    )


def test_ahrs_filter_frontier_advance_publishes_state() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(), clock=clock)

    first: AhrsOutputs = filt.handle_event(_event_at(1, 0))
    second: AhrsOutputs = filt.handle_event(_event_at(2, 0))

    assert first.frontier_advanced is True
    assert second.frontier_advanced is True
    assert first.state is not None
    assert second.state is not None
    assert first.state.state_seq == 1
    assert second.state.state_seq == 2


def test_ahrs_filter_out_of_order_does_not_publish_state() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(), clock=clock)

    first: AhrsOutputs = filt.handle_event(_event_at(2, 0))
    out_of_order: AhrsOutputs = filt.handle_event(_event_at(1, 0))
    assert first.frontier_advanced is True
    assert out_of_order.frontier_advanced is False
    assert out_of_order.state is None
    assert out_of_order.diagnostics is None
    assert out_of_order.mag_update is not None


def test_ahrs_filter_window_rejects_too_old() -> None:
    clock: FixedClock = FixedClock(now_sec=20, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(t_buffer_sec=0.5), clock=clock)

    advance: AhrsOutputs = filt.handle_event(_event_at(10, 0))
    dropped: AhrsOutputs = filt.handle_event(_event_at(9, 0))

    assert advance.frontier_advanced is True
    assert dropped.frontier_advanced is False
    assert dropped.state is None
    assert dropped.t_filter == advance.t_filter


def test_ahrs_filter_clock_jump_resets_on_large_abs_dt() -> None:
    clock: FixedClock = FixedClock(now_sec=30, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(
        config=_config(dt_clock_jump_max_sec=1.0), clock=clock
    )

    initial: AhrsOutputs = filt.handle_event(_event_at(10, 0))
    jumped: AhrsOutputs = filt.handle_event(_event_at(20, 500_000_000))

    assert initial.frontier_advanced is True
    assert jumped.frontier_advanced is False
    assert filt._t_filter is None
    assert filt._initialized is False
    assert filt._reset_count == 1
    assert filt._dropped_clock_jump_reset == 1


def test_ahrs_filter_negative_dt_raises() -> None:
    clock: FixedClock = FixedClock(now_sec=30, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(), clock=clock)
    start_time: AhrsTime = AhrsTime(sec=2, nanosec=0)
    earlier_time: AhrsTime = AhrsTime(sec=1, nanosec=0)

    filt._reset_replay_state(start_time)

    with pytest.raises(ValueError, match="dt_sec must be non-negative"):
        filt._propagate_to(earlier_time)


def test_buffer_node_count_and_span_reported() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(t_buffer_sec=5.0), clock=clock)

    first: AhrsOutputs = filt.handle_event(_event_at(1, 0))
    second: AhrsOutputs = filt.handle_event(_event_at(2, 0))

    assert first.frontier_advanced is True
    assert second.diagnostics is not None
    assert second.diagnostics.buffer_node_count == 2
    assert second.diagnostics.buffer_span_sec == 1.0


def test_replay_flag_set_after_out_of_order_event() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(t_buffer_sec=5.0), clock=clock)

    first: AhrsOutputs = filt.handle_event(_event_at(2, 0))
    assert first.diagnostics is not None
    assert first.diagnostics.replay_happened is False

    out_of_order: AhrsOutputs = filt.handle_event(_event_at(1, 0))
    assert out_of_order.frontier_advanced is False
    assert out_of_order.diagnostics is None

    second: AhrsOutputs = filt.handle_event(_event_at(3, 0))
    assert second.diagnostics is not None
    assert second.diagnostics.replay_happened is True

    third: AhrsOutputs = filt.handle_event(_event_at(4, 0))
    assert third.diagnostics is not None
    assert third.diagnostics.replay_happened is False


def test_imu_gap_detection_ignores_mag_only_nodes() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(
        config=_config(t_buffer_sec=20.0, dt_imu_max_sec=0.15),
        clock=clock,
    )

    first_imu: AhrsOutputs = filt.handle_event(_imu_event_at(1, 0))
    second_imu: AhrsOutputs = filt.handle_event(_imu_event_at(1, 100_000_000))
    mag_late: AhrsOutputs = filt.handle_event(_event_at(10, 0))
    out_of_order_imu: AhrsOutputs = filt.handle_event(_imu_event_at(1, 200_000_000))

    assert first_imu.frontier_advanced is True
    assert second_imu.frontier_advanced is True
    assert mag_late.frontier_advanced is True
    assert out_of_order_imu.frontier_advanced is False
    assert filt._dropped_imu_gap == 0


def test_mag_update_initializes_field_after_gravity() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(config=_config(t_buffer_sec=5.0), clock=clock)

    imu_outputs: AhrsOutputs = filt.handle_event(_imu_event_at(1, 0))
    assert imu_outputs.accel_update is not None
    assert imu_outputs.accel_update.accepted is True

    mag_event: AhrsEvent = _mag_event_at(2, 0, [0.3, 0.1, -0.2])
    mag_outputs: AhrsOutputs = filt.handle_event(mag_event)

    assert mag_outputs.mag_update is not None
    assert mag_outputs.mag_update.accepted is True
    assert mag_outputs.mag_update.reject_reason is None
    assert mag_outputs.state is not None
    assert any(abs(value) > 0.0 for value in mag_outputs.state.m_w_t)
