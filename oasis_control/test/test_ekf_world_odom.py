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
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfFrameOutputs
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import from_ns


_NS_PER_S: int = 1_000_000_000
_NS_PER_MS: int = 1_000_000


def _ns_from_s(seconds: int) -> int:
    return seconds * _NS_PER_S


def _ns_from_ms(milliseconds: int) -> int:
    return milliseconds * _NS_PER_MS


def _build_config() -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=_ns_from_s(1),
        epsilon_wall_future_ns=_ns_from_ms(100),
        dt_clock_jump_max_ns=_ns_from_s(5),
        dt_imu_max_ns=_ns_from_s(1),
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_ns=_ns_from_ms(10),
        checkpoint_interval_ns=_ns_from_ms(100),
        apriltag_pos_var=0.05,
        apriltag_yaw_var=0.01,
        apriltag_gate_d2=0.0,
        apriltag_reproj_rms_gate_px=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=math.pi,
        mag_alpha=0.01,
        mag_r_min=[
            1.0e-12,
            0.0,
            0.0,
            0.0,
            1.0e-12,
            0.0,
            0.0,
            0.0,
            1.0e-12,
        ],
        mag_r_max=[
            2.5e-9,
            0.0,
            0.0,
            0.0,
            2.5e-9,
            0.0,
            0.0,
            0.0,
            2.5e-9,
        ],
        mag_r0_default=[
            4.0e-10,
            0.0,
            0.0,
            0.0,
            4.0e-10,
            0.0,
            0.0,
            0.0,
            4.0e-10,
        ],
        mag_world_t=[1.0, 0.0, 0.0],
    )


def _build_camera_info() -> CameraInfoData:
    return CameraInfoData(
        frame_id="camera",
        width=640,
        height=480,
        distortion_model="",
        d=[0.0] * 5,
        k=[
            500.0,
            0.0,
            320.0,
            0.0,
            500.0,
            240.0,
            0.0,
            0.0,
            1.0,
        ],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[
            500.0,
            0.0,
            320.0,
            0.0,
            0.0,
            500.0,
            240.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
    )


def _project_tag_corners(
    *,
    pose_cam_xyz_yaw: list[float],
    tag_size_m: float,
    camera_info: CameraInfoData,
) -> list[float]:
    half_size_m: float = 0.5 * tag_size_m
    tag_corners_t: list[list[float]] = [
        [-half_size_m, -half_size_m, 0.0],
        [half_size_m, -half_size_m, 0.0],
        [half_size_m, half_size_m, 0.0],
        [-half_size_m, half_size_m, 0.0],
    ]

    x_t: float = pose_cam_xyz_yaw[0]
    y_t: float = pose_cam_xyz_yaw[1]
    z_t: float = pose_cam_xyz_yaw[2]
    yaw: float = pose_cam_xyz_yaw[3]
    cos_yaw: float = math.cos(yaw)
    sin_yaw: float = math.sin(yaw)

    fx: float = camera_info.k[0]
    fy: float = camera_info.k[4]
    cx: float = camera_info.k[2]
    cy: float = camera_info.k[5]

    corners_px: list[float] = []
    corner: list[float]
    for corner in tag_corners_t:
        x_c: float = cos_yaw * corner[0] - sin_yaw * corner[1] + x_t
        y_c: float = sin_yaw * corner[0] + cos_yaw * corner[1] + y_t
        z_c: float = corner[2] + z_t
        u_val: float = fx * (x_c / z_c) + cx
        v_val: float = fy * (y_c / z_c) + cy
        corners_px.extend([u_val, v_val])

    return corners_px


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
    *,
    pose_cam_xyz_yaw: list[float],
    tag_size_m: float,
    camera_info: CameraInfoData,
) -> AprilTagDetection:
    corners_px: list[float] = _project_tag_corners(
        pose_cam_xyz_yaw=pose_cam_xyz_yaw,
        tag_size_m=tag_size_m,
        camera_info=camera_info,
    )
    homography: list[float] = [0.0] * 9
    return AprilTagDetection(
        family="tag36h11",
        tag_id=1,
        det_index_in_msg=0,
        corners_px=corners_px,
        pose_cam_xyz_yaw=pose_cam_xyz_yaw,
        decision_margin=1.0,
        homography=homography,
    )


def _assert_close_sequence(
    *, expected: list[float], actual: list[float], tol: float
) -> None:
    assert len(actual) == len(expected)
    for exp, act in zip(expected, actual):
        assert math.isclose(act, exp, abs_tol=tol)


def _quat_multiply(lhs: list[float], rhs: list[float]) -> list[float]:
    w1: float
    x1: float
    y1: float
    z1: float
    w2: float
    x2: float
    y2: float
    z2: float
    w1, x1, y1, z1 = lhs
    w2, x2, y2, z2 = rhs
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _quat_normalize(quat: list[float]) -> list[float]:
    norm: float = math.sqrt(sum(value * value for value in quat))
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in quat]


def _rotation_matrix(quat: list[float]) -> list[list[float]]:
    w: float
    x: float
    y: float
    z: float
    w, x, y, z = _quat_normalize(quat)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def _pose_compose(
    t_ab: list[float],
    q_ab: list[float],
    t_bc: list[float],
    q_bc: list[float],
) -> tuple[list[float], list[float]]:
    rot_ab: list[list[float]] = _rotation_matrix(q_ab)
    t_ac: list[float] = [
        t_ab[0]
        + rot_ab[0][0] * t_bc[0]
        + rot_ab[0][1] * t_bc[1]
        + rot_ab[0][2] * t_bc[2],
        t_ab[1]
        + rot_ab[1][0] * t_bc[0]
        + rot_ab[1][1] * t_bc[1]
        + rot_ab[1][2] * t_bc[2],
        t_ab[2]
        + rot_ab[2][0] * t_bc[0]
        + rot_ab[2][1] * t_bc[1]
        + rot_ab[2][2] * t_bc[2],
    ]
    q_ac: list[float] = _quat_normalize(_quat_multiply(q_ab, q_bc))
    return t_ac, q_ac


def test_global_update_does_not_teleport_odom() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)

    camera_info: CameraInfoData = _build_camera_info()
    camera_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    core.process_event(camera_event)

    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(0),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(_ns_from_ms(500)),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )

    odom_before: list[float] = core.state().tolist()

    detection: AprilTagDetection = _build_apriltag_detection(
        pose_cam_xyz_yaw=[1.0, 0.0, 1.0, 0.1],
        tag_size_m=config.tag_size_m,
        camera_info=camera_info,
    )
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(500)),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[detection],
        ),
    )
    core.process_event(apriltag_event)

    odom_after: list[float] = core.state().tolist()
    _assert_close_sequence(expected=odom_before, actual=odom_after, tol=1.0e-9)

    world_odom_after: Pose3 = core.world_odom_pose()
    assert math.isfinite(float(world_odom_after.translation_m[0]))
    assert abs(float(world_odom_after.translation_m[0])) > 0.0


def test_world_correction_updates_world_odom_only() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)

    camera_info: CameraInfoData = _build_camera_info()
    camera_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    core.process_event(camera_event)

    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(0),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(_ns_from_ms(500)),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )

    odom_before: list[float] = core.state().tolist()
    world_odom_before: Pose3 = core.world_odom_pose()

    detection: AprilTagDetection = _build_apriltag_detection(
        pose_cam_xyz_yaw=[0.5, -0.2, 1.0, -0.1],
        tag_size_m=config.tag_size_m,
        camera_info=camera_info,
    )
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(500)),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[detection],
        ),
    )
    core.process_event(apriltag_event)

    odom_after: list[float] = core.state().tolist()
    _assert_close_sequence(expected=odom_before, actual=odom_after, tol=1.0e-9)

    world_odom_after: Pose3 = core.world_odom_pose()
    assert abs(float(world_odom_after.translation_m[0])) > abs(
        float(world_odom_before.translation_m[0])
    )


def test_world_base_composition_matches_outputs() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)

    camera_info: CameraInfoData = _build_camera_info()
    camera_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    core.process_event(camera_event)

    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(0),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(_ns_from_ms(500)),
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )
    core.process_event(
        EkfEvent(
            t_meas=from_ns(_ns_from_ms(500)),
            event_type=EkfEventType.APRILTAG,
            payload=AprilTagDetectionArrayData(
                frame_id="camera",
                detections=[
                    _build_apriltag_detection(
                        pose_cam_xyz_yaw=[0.2, 0.1, 1.0, 0.05],
                        tag_size_m=config.tag_size_m,
                        camera_info=camera_info,
                    )
                ],
            ),
        )
    )

    transforms: EkfFrameOutputs = core.frame_transforms()
    composed_t: list[float]
    composed_q: list[float]
    composed_t, composed_q = _pose_compose(
        transforms.t_world_odom.translation_m,
        transforms.t_world_odom.rotation_wxyz,
        transforms.t_odom_base.translation_m,
        transforms.t_odom_base.rotation_wxyz,
    )
    _assert_close_sequence(
        expected=transforms.t_world_base.translation_m,
        actual=composed_t,
        tol=1.0e-9,
    )
    _assert_close_sequence(
        expected=transforms.t_world_base.rotation_wxyz,
        actual=composed_q,
        tol=1.0e-9,
    )


def test_replay_updates_world_odom_deterministically() -> None:
    config: EkfConfig = _build_config()
    reference_core: EkfCore = EkfCore(config)
    sut_core: EkfCore = EkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)

    camera_info: CameraInfoData = _build_camera_info()
    camera_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    reference_core.process_event(camera_event)
    sut_core.process_event(camera_event)

    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )
    imu_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )
    reference_core.process_event(imu_event)
    sut_core.process_event(imu_event)

    early_event: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(100)),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[
                _build_apriltag_detection(
                    pose_cam_xyz_yaw=[0.4, 0.0, 1.0, 0.1],
                    tag_size_m=config.tag_size_m,
                    camera_info=camera_info,
                )
            ],
        ),
    )
    late_event: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_ms(200)),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(
            frame_id="camera",
            detections=[
                _build_apriltag_detection(
                    pose_cam_xyz_yaw=[0.6, 0.1, 1.0, -0.1],
                    tag_size_m=config.tag_size_m,
                    camera_info=camera_info,
                )
            ],
        ),
    )

    reference_core.process_event(early_event)
    reference_core.process_event(late_event)

    buffer.insert_event(imu_event)
    sut_core.process_event(imu_event)
    buffer.insert_event(late_event)
    sut_core.process_event(late_event)
    buffer.insert_event(early_event)
    sut_core.replay(buffer, start_time=early_event.t_meas)

    ref_world: EkfFrameOutputs = reference_core.frame_transforms()
    sut_world: EkfFrameOutputs = sut_core.frame_transforms()
    _assert_close_sequence(
        expected=ref_world.t_world_odom.translation_m,
        actual=sut_world.t_world_odom.translation_m,
        tol=1.0e-9,
    )
    _assert_close_sequence(
        expected=ref_world.t_world_odom.rotation_wxyz,
        actual=sut_world.t_world_odom.rotation_wxyz,
        tol=1.0e-9,
    )
    _assert_close_sequence(
        expected=ref_world.t_world_base.translation_m,
        actual=sut_world.t_world_base.translation_m,
        tol=1.0e-9,
    )
    _assert_close_sequence(
        expected=ref_world.t_world_base.rotation_wxyz,
        actual=sut_world.t_world_base.rotation_wxyz,
        tol=1.0e-9,
    )
