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

import math
from types import ModuleType

import pytest


np: ModuleType = pytest.importorskip(
    "numpy", reason="TODO: requires numpy for EKF tests"
)

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EventAprilTagPose
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.pose_math import quaternion_from_rpy


def _build_config(
    *,
    t_buffer_sec: float,
    checkpoint_interval_sec: float,
    apriltag_gate_d2: float,
    apriltag_pos_var: float,
    apriltag_yaw_var: float,
) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=t_buffer_sec,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=5.0,
        dt_imu_max=1.0,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_sec=0.01,
        checkpoint_interval_sec=checkpoint_interval_sec,
        apriltag_pos_var=apriltag_pos_var,
        apriltag_yaw_var=apriltag_yaw_var,
        apriltag_gate_d2=apriltag_gate_d2,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
    )


def _build_imu_sample(*, accel_body: list[float]) -> ImuSample:
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=accel_body,
        angular_velocity_cov=gyro_cov,
        linear_acceleration_cov=accel_cov,
    )


def _build_pose_covariance(*, pos_var: float, rot_var: float) -> list[float]:
    covariance: list[float] = [0.0] * 36
    for index in range(3):
        covariance[index * 6 + index] = pos_var
    for index in range(3):
        covariance[(index + 3) * 6 + (index + 3)] = rot_var
    return covariance


def _build_apriltag_pose_event(
    *,
    pose_world_xyz: list[float],
    yaw_rad: float,
    tag_id: int,
    t_meas: float,
) -> EkfEvent:
    quat: tuple[float, float, float, float] = quaternion_from_rpy(
        0.0, 0.0, yaw_rad
    )
    covariance: list[float] = _build_pose_covariance(
        pos_var=0.05,
        rot_var=0.01,
    )
    payload: EventAprilTagPose = EventAprilTagPose(
        timestamp_s=t_meas,
        p_meas_world_base_m=pose_world_xyz,
        q_meas_world_base_xyzw=list(quat),
        covariance=covariance,
        tag_id=tag_id,
        frame_id="camera",
        source_topic="apriltags",
        family="tag36h11",
        det_index_in_msg=0,
    )
    return EkfEvent(
        t_meas=from_seconds(t_meas),
        event_type=EkfEventType.APRILTAG,
        payload=payload,
    )


def _flatten_matrix(matrix: list[list[float]]) -> list[float]:
    values: list[float] = []
    row_count: int = len(matrix)
    for row_index in range(row_count):
        row_values: list[float] = matrix[row_index]
        values.extend(row_values)
    return values


def _assert_close_sequence(
    *, expected: list[float], actual: list[float], tol: float
) -> None:
    assert len(actual) == len(expected)
    index_count: int = len(expected)
    for index in range(index_count):
        expected_value: float = expected[index]
        actual_value: float = actual[index]
        assert math.isclose(actual_value, expected_value, abs_tol=tol)


def _capture_state(core: EkfCore) -> tuple[list[float], list[float]]:
    state_list: list[float] = core.state().tolist()
    covariance_list: list[list[float]] = core.covariance().tolist()
    flattened_cov: list[float] = _flatten_matrix(covariance_list)
    return state_list, flattened_cov


def test_out_of_order_apriltag_replay_matches_chronological() -> None:
    config: EkfConfig = _build_config(
        t_buffer_sec=1.0,
        checkpoint_interval_sec=0.05,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=0.05,
        apriltag_yaw_var=0.01,
    )
    reference_core: EkfCore = EkfCore(config)
    sut_core: EkfCore = EkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)

    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.5, 0.0, config.gravity_mps2]
    )
    imu_time: EkfTime = from_seconds(0.0)
    imu_event: EkfEvent = EkfEvent(
        t_meas=imu_time,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )

    early_event: EkfEvent = _build_apriltag_pose_event(
        pose_world_xyz=[0.2, -0.1, 0.0],
        yaw_rad=0.05,
        tag_id=1,
        t_meas=0.1,
    )
    late_event: EkfEvent = _build_apriltag_pose_event(
        pose_world_xyz=[0.5, 0.2, 0.0],
        yaw_rad=-0.1,
        tag_id=2,
        t_meas=0.2,
    )

    reference_core.process_event(imu_event)
    reference_core.process_event(early_event)
    reference_core.process_event(late_event)
    ref_state: list[float]
    ref_cov: list[float]
    ref_state, ref_cov = _capture_state(reference_core)

    buffer.insert_event(imu_event)
    sut_core.process_event(imu_event)
    buffer.insert_event(late_event)
    sut_core.process_event(late_event)
    buffer.insert_event(early_event)
    sut_core.replay(buffer, start_time=early_event.t_meas)
    sut_state: list[float]
    sut_cov: list[float]
    sut_state, sut_cov = _capture_state(sut_core)

    _assert_close_sequence(expected=ref_state, actual=sut_state, tol=1.0e-9)
    _assert_close_sequence(expected=ref_cov, actual=sut_cov, tol=1.0e-9)


def test_buffer_stable_ordering_for_equal_times() -> None:
    config: EkfConfig = _build_config(
        t_buffer_sec=1.0,
        checkpoint_interval_sec=0.1,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    buffer: EkfBuffer = EkfBuffer(config)

    imu_sample_a: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )
    imu_sample_b: ImuSample = _build_imu_sample(
        accel_body=[0.1, 0.0, config.gravity_mps2]
    )
    imu_sample_c: ImuSample = _build_imu_sample(
        accel_body=[0.2, 0.0, config.gravity_mps2]
    )

    event_first: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample_a, calibration=None),
    )
    event_middle: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.5),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample_b, calibration=None),
    )
    event_second: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample_c, calibration=None),
    )

    buffer.insert_event(event_first)
    buffer.insert_event(event_middle)
    buffer.insert_event(event_second)

    ordered_events: list[EkfEvent] = list(buffer.iter_events())

    assert len(ordered_events) == 3
    assert ordered_events[0] is event_middle
    assert ordered_events[1] is event_first
    assert ordered_events[2] is event_second
