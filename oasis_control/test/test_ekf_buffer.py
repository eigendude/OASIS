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

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import from_ns


_NS_PER_S: int = 1_000_000_000


def _build_config(*, t_buffer_ns: int = 10) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=t_buffer_ns,
        epsilon_wall_future_ns=100,
        dt_clock_jump_max_ns=_NS_PER_S,
        dt_imu_max_ns=50,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_ns=10,
        checkpoint_interval_ns=100,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=0.0,
        apriltag_reproj_rms_gate_px=0.0,
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


def _build_event(t_meas: EkfTime) -> EkfEvent:
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )


def test_buffer_orders_events_with_same_timestamp() -> None:
    config: EkfConfig = _build_config()
    buffer: EkfBuffer = EkfBuffer(config)
    timestamp: EkfTime = from_ns(5)

    first_event: EkfEvent = _build_event(timestamp)
    second_event: EkfEvent = _build_event(timestamp)

    buffer.insert_event(first_event)
    buffer.insert_event(second_event)

    events: list[EkfEvent] = list(buffer.iter_events())

    assert events == [first_event, second_event]


def test_buffer_evicts_old_events() -> None:
    config: EkfConfig = _build_config(t_buffer_ns=10)
    buffer: EkfBuffer = EkfBuffer(config)

    event_early: EkfEvent = _build_event(from_ns(0))
    event_boundary: EkfEvent = _build_event(from_ns(10))
    event_recent: EkfEvent = _build_event(from_ns(15))

    buffer.insert_event(event_early)
    buffer.insert_event(event_boundary)
    buffer.insert_event(event_recent)

    buffer.evict(15)

    events: list[EkfEvent] = list(buffer.iter_events())

    assert events == [event_boundary, event_recent]
