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
from typing import Any

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import FrameTransform
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.se3 import pose_compose


def _build_config() -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=2.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=5.0,
        dt_imu_max=1.0,
        pos_var=0.5,
        vel_var=0.5,
        ang_var=0.2,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_sec=0.01,
        checkpoint_interval_sec=0.2,
        apriltag_pos_var=0.05,
        apriltag_yaw_var=0.01,
        apriltag_gate_d2=0.0,
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


def _build_camera_info() -> CameraInfoData:
    return CameraInfoData(
        frame_id="camera",
        width=640,
        height=480,
        distortion_model="plumb_bob",
        d=[0.0] * 5,
        k=[0.0] * 9,
        r=[0.0] * 9,
        p=[0.0] * 12,
    )


def _build_apriltag_detection(*, pose_world_xyz_yaw: list[float]) -> AprilTagDetection:
    corners_px: list[float] = [0.0] * 8
    homography: list[float] = [0.0] * 9
    return AprilTagDetection(
        family="tag36h11",
        tag_id=1,
        det_index_in_msg=0,
        corners_px=corners_px,
        pose_world_xyz_yaw=pose_world_xyz_yaw,
        decision_margin=1.0,
        homography=homography,
    )


def _frame_translation(transform: FrameTransform) -> list[float]:
    return [
        float(transform.translation_m[0]),
        float(transform.translation_m[1]),
        float(transform.translation_m[2]),
    ]


def _assert_close(*, expected: list[float], actual: list[float], tol: float) -> None:
    assert len(expected) == len(actual)
    index_count: int = len(expected)
    index: int
    for index in range(index_count):
        assert math.isclose(actual[index], expected[index], abs_tol=tol)


def _run_with_buffer(
    *, config: EkfConfig, events: list[EkfEvent]
) -> tuple[FrameTransform, FrameTransform]:
    core: EkfCore = EkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)

    event: EkfEvent
    for event in events:
        buffer.insert_event(event)
        if core.is_out_of_order(event.t_meas):
            core.replay(buffer, event.t_meas)
        else:
            core.process_event(event)

        frontier_time: EkfTime | None = core.frontier_time()
        if frontier_time is not None:
            buffer.evict(to_ns(frontier_time))

    _odom_base: FrameTransform
    world_odom: FrameTransform
    world_base: FrameTransform
    _odom_base, world_odom, world_base = core.current_transforms()
    return world_odom, world_base


def test_global_update_keeps_odom_continuous() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)

    camera_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=_build_camera_info(),
    )
    imu_event_0: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(
            imu=_build_imu_sample(accel_body=[0.5, 0.0, config.gravity_mps2]),
            calibration=None,
        ),
    )
    imu_event_1: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.1),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(
            imu=_build_imu_sample(accel_body=[0.5, 0.0, config.gravity_mps2]),
            calibration=None,
        ),
    )
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.1),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[
                _build_apriltag_detection(pose_world_xyz_yaw=[1.0, 0.2, 0.0, 0.3])
            ],
        ),
    )

    core.process_event(camera_event)
    core.process_event(imu_event_0)
    core.process_event(imu_event_1)
    odom_before: FrameTransform
    world_odom_before: FrameTransform
    world_base_before: FrameTransform
    odom_before, world_odom_before, world_base_before = core.current_transforms()

    core.process_event(apriltag_event)
    odom_after: FrameTransform
    world_odom_after: FrameTransform
    world_base_after: FrameTransform
    odom_after, world_odom_after, world_base_after = core.current_transforms()

    _assert_close(
        expected=_frame_translation(odom_before),
        actual=_frame_translation(odom_after),
        tol=1.0e-9,
    )
    assert _frame_translation(world_odom_before) != _frame_translation(world_odom_after)
    assert _frame_translation(world_base_before) != _frame_translation(world_base_after)


def test_world_base_is_composed_from_world_odom_and_odom_base() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)

    core.process_event(
        EkfEvent(
            t_meas=from_seconds(0.0),
            event_type=EkfEventType.CAMERA_INFO,
            payload=_build_camera_info(),
        )
    )
    core.process_event(
        EkfEvent(
            t_meas=from_seconds(0.0),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(
                imu=_build_imu_sample(accel_body=[0.3, 0.0, config.gravity_mps2]),
                calibration=None,
            ),
        )
    )
    core.process_event(
        EkfEvent(
            t_meas=from_seconds(0.2),
            event_type=EkfEventType.APRILTAG,
            payload=AprilTagDetectionArrayData(
                frame_id="camera",
                detections=[
                    _build_apriltag_detection(pose_world_xyz_yaw=[0.4, 0.0, 0.0, 0.1])
                ],
            ),
        )
    )

    odom_base: FrameTransform
    world_odom: FrameTransform
    world_base: FrameTransform
    odom_base, world_odom, world_base = core.current_transforms()

    composed_t: list[float]
    composed_q: list[float]
    composed_t_array: Any
    composed_q_array: Any
    composed_t_array, composed_q_array = pose_compose(
        world_odom.translation_m,
        world_odom.rotation_wxyz,
        odom_base.translation_m,
        odom_base.rotation_wxyz,
    )
    composed_t = composed_t_array.tolist()
    composed_q = composed_q_array.tolist()

    _assert_close(expected=world_base.translation_m, actual=composed_t, tol=1.0e-9)
    _assert_close(expected=world_base.rotation_wxyz, actual=composed_q, tol=1.0e-9)


def test_replay_order_preserves_world_alignment() -> None:
    config: EkfConfig = _build_config()

    camera_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=_build_camera_info(),
    )
    imu_event_0: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(
            imu=_build_imu_sample(accel_body=[0.2, 0.0, config.gravity_mps2]),
            calibration=None,
        ),
    )
    imu_event_1: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.2),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(
            imu=_build_imu_sample(accel_body=[0.2, 0.0, config.gravity_mps2]),
            calibration=None,
        ),
    )
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.15),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[
                _build_apriltag_detection(pose_world_xyz_yaw=[0.6, 0.1, 0.0, -0.2])
            ],
        ),
    )

    chronological_events: list[EkfEvent] = [
        camera_event,
        imu_event_0,
        apriltag_event,
        imu_event_1,
    ]
    out_of_order_events: list[EkfEvent] = [
        camera_event,
        imu_event_0,
        imu_event_1,
        apriltag_event,
    ]

    world_odom_a: FrameTransform
    world_base_a: FrameTransform
    world_odom_b: FrameTransform
    world_base_b: FrameTransform
    world_odom_a, world_base_a = _run_with_buffer(
        config=config, events=chronological_events
    )
    world_odom_b, world_base_b = _run_with_buffer(
        config=config, events=out_of_order_events
    )

    _assert_close(
        expected=_frame_translation(world_odom_a),
        actual=_frame_translation(world_odom_b),
        tol=1.0e-9,
    )
    _assert_close(
        expected=_frame_translation(world_base_a),
        actual=_frame_translation(world_base_b),
        tol=1.0e-9,
    )
