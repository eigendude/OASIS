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
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import EkfStateIndex
from oasis_control.localization.ekf.ekf_state import TagKey
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
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
        apriltag_pos_var=0.01,
        apriltag_yaw_var=0.01,
        apriltag_gate_d2=0.0,
        apriltag_reproj_rms_gate_px=0.0,
        tag_size_m=0.2,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
        extrinsic_prior_sigma_t_m=1.0,
        extrinsic_prior_sigma_rot_rad=3.141592653589793,
        mag_alpha=0.01,
        mag_r_min_t2=[1.0e-12, 1.0e-12, 1.0e-12],
        mag_r_max_t2=[2.5e-9, 2.5e-9, 2.5e-9],
        mag_r0_t2=[4.0e-10, 4.0e-10, 4.0e-10],
        mag_world_t=[1.0, 0.0, 0.0],
    )


def _build_camera_info(*, frame_id: str) -> CameraInfoData:
    fx: float = 600.0
    fy: float = 600.0
    cx: float = 320.0
    cy: float = 240.0

    k: list[float] = [
        fx,
        0.0,
        cx,
        0.0,
        fy,
        cy,
        0.0,
        0.0,
        1.0,
    ]
    r: list[float] = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    p: list[float] = [
        fx,
        0.0,
        cx,
        0.0,
        0.0,
        fy,
        cy,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return CameraInfoData(
        frame_id=frame_id,
        width=640,
        height=480,
        distortion_model="none",
        d=[],
        k=k,
        r=r,
        p=p,
    )


def _build_imu_packet() -> EkfImuPacket:
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    imu_sample: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, 0.0],
        angular_velocity_cov=gyro_cov,
        linear_acceleration_cov=accel_cov,
    )
    return EkfImuPacket(imu=imu_sample, calibration=None)


def _rotation_matrix(*, roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr: float = math.cos(roll)
    sr: float = math.sin(roll)
    cp: float = math.cos(pitch)
    sp: float = math.sin(pitch)
    cy: float = math.cos(yaw)
    sy: float = math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rotate_point(*, rotation: list[list[float]], point: list[float]) -> list[float]:
    x: float = (
        rotation[0][0] * point[0]
        + rotation[0][1] * point[1]
        + rotation[0][2] * point[2]
    )
    y: float = (
        rotation[1][0] * point[0]
        + rotation[1][1] * point[1]
        + rotation[1][2] * point[2]
    )
    z: float = (
        rotation[2][0] * point[0]
        + rotation[2][1] * point[1]
        + rotation[2][2] * point[2]
    )
    return [x, y, z]


def _project_tag_corners(
    *, tag_pose_c: list[float], tag_size_m: float, camera_info: CameraInfoData
) -> list[float]:
    # Half tag edge length in meters for corner offsets
    half_size_m: float = 0.5 * tag_size_m
    corners_t: list[list[float]] = [
        [-half_size_m, -half_size_m, 0.0],
        [half_size_m, -half_size_m, 0.0],
        [half_size_m, half_size_m, 0.0],
        [-half_size_m, half_size_m, 0.0],
    ]

    translation_c: list[float] = tag_pose_c[:3]
    rotation: list[list[float]] = _rotation_matrix(
        roll=tag_pose_c[3], pitch=tag_pose_c[4], yaw=tag_pose_c[5]
    )

    fx: float = camera_info.k[0]
    fy: float = camera_info.k[4]
    cx: float = camera_info.k[2]
    cy: float = camera_info.k[5]

    corners_px: list[float] = []
    corner: list[float]
    for corner in corners_t:
        rotated: list[float] = _rotate_point(rotation=rotation, point=corner)
        corner_c: list[float] = [
            rotated[0] + translation_c[0],
            rotated[1] + translation_c[1],
            rotated[2] + translation_c[2],
        ]
        z: float = corner_c[2]
        x_norm: float = corner_c[0] / z
        y_norm: float = corner_c[1] / z
        u: float = fx * x_norm + cx
        v: float = fy * y_norm + cy
        corners_px.extend([u, v])
    return corners_px


def _build_detection(
    *, tag_pose_c: list[float], tag_size_m: float, camera_info: CameraInfoData
) -> AprilTagDetection:
    corners_px: list[float] = _project_tag_corners(
        tag_pose_c=tag_pose_c, tag_size_m=tag_size_m, camera_info=camera_info
    )
    homography: list[float] = [0.0] * 9
    return AprilTagDetection(
        family="tag36h11",
        tag_id=5,
        det_index_in_msg=0,
        corners_px=corners_px,
        pose_cam_xyz_yaw=[
            tag_pose_c[0],
            tag_pose_c[1],
            tag_pose_c[2],
            tag_pose_c[5],
        ],
        decision_margin=1.0,
        homography=homography,
    )


def _run_pipeline(
    *, tag_pose_c: list[float]
) -> tuple[EkfAprilTagUpdateData, EkfStateIndex]:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    frame_id: str = "camera"
    camera_info: CameraInfoData = _build_camera_info(frame_id=frame_id)

    t_meas: EkfTime = from_ns(0)
    camera_event: EkfEvent = EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    core.process_event(camera_event)

    imu_event: EkfEvent = EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.IMU,
        payload=_build_imu_packet(),
    )
    core.process_event(imu_event)

    detection: AprilTagDetection = _build_detection(
        tag_pose_c=tag_pose_c, tag_size_m=config.tag_size_m, camera_info=camera_info
    )
    apriltag_data: AprilTagDetectionArrayData = AprilTagDetectionArrayData(
        frame_id=frame_id,
        detections=[detection],
    )
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.APRILTAG,
        payload=apriltag_data,
    )

    outputs: EkfOutputs = core.process_event(apriltag_event)
    assert outputs.apriltag_update is not None
    return outputs.apriltag_update, core.state_index()


def _assert_close_sequence(
    *, expected: list[float], actual: list[float], tol: float
) -> None:
    assert len(expected) == len(actual)
    index: int
    for index in range(len(expected)):
        assert math.isclose(actual[index], expected[index], abs_tol=tol)


def test_apriltag_pipeline_integration() -> None:
    """
    Exercise the EKF AprilTag pipeline entrypoint end to end
    """

    # This exercises camera info ingestion, reprojection refinement, and the
    # AprilTag update report produced by EkfCore.process_event
    tag_pose_c: list[float] = [0.15, -0.1, 1.2, 0.0, 0.0, 0.25]
    update_first: EkfAprilTagUpdateData
    state_index_first: EkfStateIndex
    update_first, state_index_first = _run_pipeline(tag_pose_c=tag_pose_c)

    detection_update: EkfUpdateData = update_first.detections[0].update
    assert detection_update.accepted

    # Allow tiny numerical residue in reprojection RMS, pixels
    assert detection_update.reproj_rms_px < 1.0e-4

    expected_z: list[float] = [
        tag_pose_c[0],
        tag_pose_c[1],
        tag_pose_c[2],
        tag_pose_c[5],
    ]
    _assert_close_sequence(expected=expected_z, actual=detection_update.z, tol=1.0e-6)

    tag_key: TagKey = TagKey(family="tag36h11", tag_id=5)
    assert tag_key in state_index_first.landmarks

    update_second: EkfAprilTagUpdateData
    state_index_second: EkfStateIndex
    update_second, state_index_second = _run_pipeline(tag_pose_c=tag_pose_c)
    detection_update_second: EkfUpdateData = update_second.detections[0].update

    _assert_close_sequence(
        expected=detection_update.z, actual=detection_update_second.z, tol=1.0e-9
    )
    assert math.isclose(
        detection_update.reproj_rms_px,
        detection_update_second.reproj_rms_px,
        abs_tol=1.0e-12,
    )
    assert state_index_first.landmarks.keys() == state_index_second.landmarks.keys()
