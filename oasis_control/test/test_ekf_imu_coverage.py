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

from typing import Iterable

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_seconds


def _build_config(*, dt_imu_max: float) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=1.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=1.0,
        dt_imu_max=dt_imu_max,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_sec=0.01,
        checkpoint_interval_sec=0.25,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=3.141592653589793,
    )


def _build_imu_sample() -> ImuSample:
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, 9.81],
        angular_velocity_cov=gyro_cov,
        linear_acceleration_cov=accel_cov,
    )


def _build_mag_sample() -> MagSample:
    mag_cov: list[float] = [0.0] * 9
    return MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=mag_cov,
    )


def _build_imu_event(t_meas: EkfTime) -> EkfEvent:
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )


def _build_mag_event(t_meas: EkfTime) -> EkfEvent:
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )


def _collect_mag_updates(outputs: Iterable[EkfOutputs]) -> list[EkfUpdateData]:
    updates: list[EkfUpdateData] = []
    for output in outputs:
        if output.mag_update is not None:
            updates.append(output.mag_update)
    return updates


def _run_replay(events: list[EkfEvent], config: EkfConfig) -> list[EkfUpdateData]:
    buffer: EkfBuffer = EkfBuffer(config)
    core: EkfCore = EkfCore(config)
    for event in events:
        buffer.insert_event(event)
    outputs: list[EkfOutputs] = core.replay(buffer)
    return _collect_mag_updates(outputs)


def test_mag_imu_coverage_accepts_dense_samples() -> None:
    config: EkfConfig = _build_config(dt_imu_max=0.05)
    events: list[EkfEvent] = [
        _build_imu_event(from_seconds(0.0)),
        _build_imu_event(from_seconds(0.01)),
        _build_mag_event(from_seconds(0.015)),
        _build_imu_event(from_seconds(0.02)),
    ]

    updates: list[EkfUpdateData] = _run_replay(events, config)

    assert len(updates) == 1
    assert updates[0].accepted


def test_mag_imu_coverage_rejects_gap() -> None:
    config: EkfConfig = _build_config(dt_imu_max=0.05)
    events: list[EkfEvent] = [
        _build_imu_event(from_seconds(0.0)),
        _build_imu_event(from_seconds(0.01)),
        _build_mag_event(from_seconds(0.20)),
        _build_imu_event(from_seconds(0.50)),
    ]

    updates: list[EkfUpdateData] = _run_replay(events, config)

    assert len(updates) == 1
    assert not updates[0].accepted
    assert updates[0].reject_reason == "imu_gap"


def test_mag_imu_coverage_recovers_after_gap() -> None:
    config: EkfConfig = _build_config(dt_imu_max=0.05)
    events: list[EkfEvent] = [
        _build_imu_event(from_seconds(0.0)),
        _build_imu_event(from_seconds(0.01)),
        _build_mag_event(from_seconds(0.20)),
        _build_imu_event(from_seconds(0.21)),
        _build_imu_event(from_seconds(0.22)),
        _build_imu_event(from_seconds(0.23)),
        _build_mag_event(from_seconds(0.235)),
    ]

    updates: list[EkfUpdateData] = _run_replay(events, config)

    assert len(updates) == 2
    assert not updates[0].accepted
    assert updates[0].reject_reason == "imu_gap"
    assert updates[1].accepted


def test_out_of_order_replay_rejects_gap() -> None:
    config: EkfConfig = _build_config(dt_imu_max=0.05)
    mag_event: EkfEvent = _build_mag_event(from_seconds(0.20))
    imu_events: list[EkfEvent] = [
        _build_imu_event(from_seconds(0.0)),
        _build_imu_event(from_seconds(0.01)),
        _build_imu_event(from_seconds(0.50)),
    ]
    events: list[EkfEvent] = [mag_event] + imu_events

    updates: list[EkfUpdateData] = _run_replay(events, config)

    assert len(updates) == 1
    assert not updates[0].accepted
    assert updates[0].reject_reason == "imu_gap"


def test_out_of_order_replay_accepts_covered_interval() -> None:
    config: EkfConfig = _build_config(dt_imu_max=0.05)
    mag_event: EkfEvent = _build_mag_event(from_seconds(0.20))
    imu_events: list[EkfEvent] = [
        _build_imu_event(from_seconds(0.0)),
        _build_imu_event(from_seconds(0.05)),
        _build_imu_event(from_seconds(0.10)),
        _build_imu_event(from_seconds(0.15)),
        _build_imu_event(from_seconds(0.19)),
    ]
    events: list[EkfEvent] = [mag_event] + imu_events

    updates: list[EkfUpdateData] = _run_replay(events, config)

    assert len(updates) == 1
    assert updates[0].accepted
