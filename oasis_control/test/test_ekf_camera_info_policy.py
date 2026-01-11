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
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfAprilTagUpdateData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData


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
    )


def _build_camera_info(
    *,
    frame_id: str = "camera",
    k: list[float] | None = None,
    d: list[float] | None = None,
    distortion_model: str = "plumb_bob",
    p: list[float] | None = None,
) -> CameraInfoData:
    k_values: list[float] = (
        [
            500.0,
            0.0,
            320.0,
            0.0,
            500.0,
            240.0,
            0.0,
            0.0,
            1.0,
        ]
        if k is None
        else k
    )
    d_values: list[float] = [0.01, -0.02, 0.0, 0.0, 0.0] if d is None else d
    p_values: list[float] = (
        [
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
        ]
        if p is None
        else p
    )
    return CameraInfoData(
        frame_id=frame_id,
        width=640,
        height=480,
        distortion_model=distortion_model,
        d=d_values,
        k=k_values,
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=p_values,
    )


def _build_detection_array(*, frame_id: str) -> AprilTagDetectionArrayData:
    detection: AprilTagDetection = AprilTagDetection(
        family="tag36h11",
        tag_id=1,
        det_index_in_msg=0,
        corners_px=[0.0] * 8,
        pose_cam_xyz_yaw=[0.0, 0.0, 1.0, 0.0],
        decision_margin=1.0,
        homography=[0.0] * 9,
    )
    return AprilTagDetectionArrayData(frame_id=frame_id, detections=[detection])


def _assert_single_rejection(update: EkfAprilTagUpdateData, *, reason: str) -> None:
    assert len(update.detections) == 1
    detection_update: EkfUpdateData = update.detections[0].update
    assert detection_update.accepted is False
    assert detection_update.reject_reason == reason


def test_apriltag_rejected_until_camera_info_cached() -> None:
    core: EkfCore = EkfCore(_build_config())
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=0, nanosec=0),
        event_type=EkfEventType.APRILTAG,
        payload=_build_detection_array(frame_id="camera"),
    )

    outputs: EkfOutputs = core.process_event(apriltag_event)

    assert outputs.apriltag_update is not None
    _assert_single_rejection(outputs.apriltag_update, reason="No camera_info cached")


def test_apriltag_frame_mismatch_rejected() -> None:
    core: EkfCore = EkfCore(_build_config())
    camera_info_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=0, nanosec=0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=_build_camera_info(frame_id="camera"),
    )
    core.process_event(camera_info_event)

    apriltag_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=1, nanosec=0),
        event_type=EkfEventType.APRILTAG,
        payload=_build_detection_array(frame_id="other_camera"),
    )
    outputs: EkfOutputs = core.process_event(apriltag_event)

    assert outputs.apriltag_update is not None
    _assert_single_rejection(outputs.apriltag_update, reason="AprilTag frame mismatch")


def test_camera_info_mismatch_ignored() -> None:
    core: EkfCore = EkfCore(_build_config())
    camera_info: CameraInfoData = _build_camera_info()
    camera_info_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=0, nanosec=0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    core.process_event(camera_info_event)

    updated_k: list[float] = camera_info.k.copy()
    updated_k[0] = 505.0
    mismatched_info: CameraInfoData = _build_camera_info(k=updated_k)
    mismatch_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=1, nanosec=0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=mismatched_info,
    )
    core.process_event(mismatch_event)

    second_mismatched_info: CameraInfoData = _build_camera_info(
        d=[0.02, -0.01, 0.0, 0.0, 0.0]
    )
    second_mismatch_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=2, nanosec=0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=second_mismatched_info,
    )
    core.process_event(second_mismatch_event)

    assert core._camera_info == camera_info
    assert core._camera_info_mismatch_count == 1
