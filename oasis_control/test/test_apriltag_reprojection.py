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

from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
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


def _project_tag_corners(
    *,
    pose_c: list[float],
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

    x_t: float = pose_c[0]
    y_t: float = pose_c[1]
    z_t: float = pose_c[2]
    roll: float = pose_c[3]
    pitch: float = pose_c[4]
    yaw: float = pose_c[5]
    rot: list[list[float]] = _rotation_matrix(roll, pitch, yaw)

    fx: float = camera_info.k[0]
    fy: float = camera_info.k[4]
    cx: float = camera_info.k[2]
    cy: float = camera_info.k[5]

    corners_px: list[float] = []
    corner: list[float]
    for corner in tag_corners_t:
        x_c: float = (
            rot[0][0] * corner[0] + rot[0][1] * corner[1] + rot[0][2] * corner[2] + x_t
        )
        y_c: float = (
            rot[1][0] * corner[0] + rot[1][1] * corner[1] + rot[1][2] * corner[2] + y_t
        )
        z_c: float = (
            rot[2][0] * corner[0] + rot[2][1] * corner[1] + rot[2][2] * corner[2] + z_t
        )
        u_val: float = fx * (x_c / z_c) + cx
        v_val: float = fy * (y_c / z_c) + cy
        corners_px.extend([u_val, v_val])

    return corners_px


def _rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _build_camera_info() -> CameraInfoData:
    return CameraInfoData(
        frame_id="camera",
        width=640,
        height=480,
        distortion_model="",
        d=[0.0] * 5,
        k=[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
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


def test_reprojection_refines_pose() -> None:
    model: AprilTagMeasurementModel = AprilTagMeasurementModel()
    camera_info: CameraInfoData = _build_camera_info()
    model.set_camera_info(camera_info)

    tag_size_m: float = 0.2
    pose_gt: list[float] = [0.1, -0.05, 1.2, 0.05, -0.03, 0.25]
    corners_px: list[float] = _project_tag_corners(
        pose_c=pose_gt,
        tag_size_m=tag_size_m,
        camera_info=camera_info,
    )
    noise: list[float] = [0.2, -0.1, 0.1, -0.2, -0.15, 0.05, 0.1, -0.05]
    noisy_corners: list[float] = [
        value + noise[index] for index, value in enumerate(corners_px)
    ]

    detection: AprilTagDetection = AprilTagDetection(
        family="tag36h11",
        tag_id=1,
        det_index_in_msg=0,
        corners_px=noisy_corners,
        pose_world_xyz_yaw=None,
        decision_margin=1.0,
        homography=[0.0] * 9,
    )

    initial_pose: list[float] = [0.15, -0.02, 1.35, 0.0, 0.0, 0.1]
    refined_pose = model.refine_pose_from_corners(initial_pose, detection, tag_size_m)
    assert refined_pose is not None

    initial_residuals = model.reprojection_residuals(
        initial_pose, detection, tag_size_m
    )
    refined_residuals = model.reprojection_residuals(
        refined_pose, detection, tag_size_m
    )
    assert initial_residuals is not None
    assert refined_residuals is not None

    initial_rms: float = _rms(initial_residuals[2].tolist())
    refined_rms: float = _rms(refined_residuals[2].tolist())

    assert refined_rms < initial_rms
