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

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfStateIndex
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.ekf_types import to_ns


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
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=3.141592653589793,
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


def _build_apriltag_detection(
    *, pose_world_xyz_yaw: list[float], tag_id: int
) -> AprilTagDetection:
    corners_px: list[float] = [0.0] * 8
    homography: list[float] = [0.0] * 9
    return AprilTagDetection(
        family="tag36h11",
        tag_id=tag_id,
        det_index_in_msg=0,
        corners_px=corners_px,
        pose_world_xyz_yaw=pose_world_xyz_yaw,
        decision_margin=1.0,
        homography=homography,
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


def _capture_state(
    core: EkfCore,
) -> tuple[list[float], list[float], list[float], list[float]]:
    state_list: list[float] = core.state().tolist()
    covariance_list: list[list[float]] = core.covariance().tolist()
    flattened_cov: list[float] = _flatten_matrix(covariance_list)
    world_odom: Pose3 = core.world_odom_pose()
    world_odom_list: list[float] = (
        world_odom.translation_m.tolist() + world_odom.rotation_wxyz.tolist()
    )
    world_odom_cov_list: list[list[float]] = core.world_odom_covariance().tolist()
    flattened_world_cov: list[float] = _flatten_matrix(world_odom_cov_list)
    return state_list, flattened_cov, world_odom_list, flattened_world_cov


def test_process_noise_coefficients() -> None:
    config: EkfConfig = _build_config(
        t_buffer_sec=1.0,
        checkpoint_interval_sec=0.1,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    core: EkfCore = EkfCore(config)
    dt: float = 0.2
    noise: list[list[float]] = core.process_noise(dt).tolist()

    accel_noise_var: float = config.accel_noise_var
    gyro_noise_var: float = config.gyro_noise_var

    # Continuous-time white-noise accel model for x=[p,v]:
    # Qpp = q_a * dt^4 / 4, Qpv = q_a * dt^3 / 2, Qvv = q_a * dt^2
    pos_noise: float = accel_noise_var * (dt**4) / 4.0
    pos_vel_noise: float = accel_noise_var * (dt**3) / 2.0
    vel_noise: float = accel_noise_var * (dt**2)

    # White gyro noise integrated to angle: Qθθ = q_g * dt^2
    ang_noise: float = gyro_noise_var * (dt**2)

    # Dimension of translation error block (x, y, z) in pose slice
    TRANSLATION_DIM: int = 3

    # Dimension of rotation error block (roll, pitch, yaw) in pose slice
    ROTATION_DIM: int = 3

    state_index: EkfStateIndex = core.state_index()
    pose_slice: slice = state_index.pose
    vel_slice: slice = state_index.velocity

    # Pose error slice is [delta_translation(3), delta_rotation(3)] in that order.
    # Indices are derived from EkfStateIndex so extra blocks won't break this test.
    rot_start: int = pose_slice.start + TRANSLATION_DIM

    axis_count: int = min(TRANSLATION_DIM, ROTATION_DIM)
    axis_index: int
    for axis_index in range(axis_count):
        pos_index: int = pose_slice.start + axis_index
        vel_index: int = vel_slice.start + axis_index
        ang_index: int = rot_start + axis_index
        assert math.isclose(noise[pos_index][pos_index], pos_noise, abs_tol=1.0e-12)
        assert math.isclose(noise[pos_index][vel_index], pos_vel_noise, abs_tol=1.0e-12)
        assert math.isclose(noise[vel_index][pos_index], pos_vel_noise, abs_tol=1.0e-12)
        assert math.isclose(noise[vel_index][vel_index], vel_noise, abs_tol=1.0e-12)
        assert math.isclose(noise[ang_index][ang_index], ang_noise, abs_tol=1.0e-12)


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

    early_detection: AprilTagDetection = _build_apriltag_detection(
        pose_world_xyz_yaw=[0.2, -0.1, 0.0, 0.05],
        tag_id=1,
    )
    early_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.1),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[early_detection],
        ),
    )

    late_detection: AprilTagDetection = _build_apriltag_detection(
        pose_world_xyz_yaw=[0.5, 0.2, 0.0, -0.1],
        tag_id=2,
    )
    late_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.2),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[late_detection],
        ),
    )

    reference_core.process_event(imu_event)
    reference_core.process_event(early_event)
    reference_core.process_event(late_event)
    ref_state: list[float]
    ref_cov: list[float]
    ref_world_odom: list[float]
    ref_world_cov: list[float]
    ref_state, ref_cov, ref_world_odom, ref_world_cov = _capture_state(reference_core)

    buffer.insert_event(imu_event)
    sut_core.process_event(imu_event)
    buffer.insert_event(late_event)
    sut_core.process_event(late_event)
    buffer.insert_event(early_event)
    sut_core.replay(buffer, start_time=early_event.t_meas)
    sut_state: list[float]
    sut_cov: list[float]
    sut_world_odom: list[float]
    sut_world_cov: list[float]
    sut_state, sut_cov, sut_world_odom, sut_world_cov = _capture_state(sut_core)

    _assert_close_sequence(expected=ref_state, actual=sut_state, tol=1.0e-9)
    _assert_close_sequence(expected=ref_cov, actual=sut_cov, tol=1.0e-9)
    _assert_close_sequence(expected=ref_world_odom, actual=sut_world_odom, tol=1.0e-9)
    _assert_close_sequence(expected=ref_world_cov, actual=sut_world_cov, tol=1.0e-9)


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


def test_buffer_too_old_relies_on_filter_frontier() -> None:
    config: EkfConfig = _build_config(
        t_buffer_sec=2.0,
        checkpoint_interval_sec=0.1,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    buffer: EkfBuffer = EkfBuffer(config)

    t_meas: EkfTime = from_seconds(5.0)
    assert buffer.too_old(t_meas, t_filter_ns=None) is False

    t_filter_ns: int = to_ns(from_seconds(10.0))
    assert buffer.too_old(from_seconds(7.0), t_filter_ns=t_filter_ns) is True
    assert buffer.too_old(from_seconds(8.0), t_filter_ns=t_filter_ns) is False


def test_replay_eviction_uses_frontier_time() -> None:
    config: EkfConfig = _build_config(
        t_buffer_sec=0.5,
        checkpoint_interval_sec=0.1,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    core: EkfCore = EkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)

    imu_time: EkfTime = from_seconds(1.0)
    imu_event: EkfEvent = EkfEvent(
        t_meas=imu_time,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(
            imu=_build_imu_sample(accel_body=[0.0, 0.0, 9.81]), calibration=None
        ),
    )
    camera_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(10.0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=CameraInfoData(
            frame_id="camera",
            width=640,
            height=480,
            distortion_model="plumb_bob",
            d=[0.0] * 5,
            k=[0.0] * 9,
            r=[0.0] * 9,
            p=[0.0] * 12,
        ),
    )

    buffer.insert_event(camera_event)
    buffer.insert_event(imu_event)

    core.replay(buffer)
    frontier_time: EkfTime | None = core.frontier_time()
    assert frontier_time is not None and frontier_time == imu_time

    buffer.evict(to_ns(frontier_time))

    remaining_events: list[EkfEvent] = list(buffer.iter_events())
    remaining_times: list[int] = [to_ns(event.t_meas) for event in remaining_events]
    assert len(remaining_times) == 2
    assert remaining_times[0] == to_ns(imu_time)
