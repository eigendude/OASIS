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

from collections.abc import Iterator
from contextlib import contextmanager
from types import ModuleType

import pytest


rclpy: ModuleType = pytest.importorskip(
    "rclpy", reason="TODO: requires rclpy for node integration tests"
)

from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.nodes.ekf_localizer_node import IMU_RAW_TOPIC
from oasis_control.nodes.ekf_localizer_node import MAG_TOPIC
from oasis_control.nodes.ekf_localizer_node import EkfLocalizerNode


@contextmanager
def _node_context() -> Iterator[EkfLocalizerNode]:
    rclpy.init(args=None)
    node: EkfLocalizerNode = EkfLocalizerNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


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


def test_backward_clock_jump_resets_buffer() -> None:
    with _node_context() as node:
        imu_sample: ImuSample = _build_imu_sample()
        event_first: EkfEvent = EkfEvent(
            t_meas=from_seconds(100.0),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
        node._process_event(event_first, IMU_RAW_TOPIC)

        event_jump: EkfEvent = EkfEvent(
            t_meas=from_seconds(1.0),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
        node._process_event(event_jump, IMU_RAW_TOPIC)

        assert node._buffer.latest_time() == 1.0
        assert len(list(node._buffer.iter_events())) == 1


def test_future_stamp_gating_rejects_event() -> None:
    with _node_context() as node:
        now_s: float = float(node.get_clock().now().nanoseconds) * 1.0e-9
        future_time: float = now_s + node._eps_wall_future + 1.0
        event_future: EkfEvent = EkfEvent(
            t_meas=from_seconds(future_time),
            event_type=EkfEventType.MAG,
            payload=_build_mag_sample(),
        )
        node._process_event(event_future, MAG_TOPIC)

        assert node._buffer.latest_time() is None
