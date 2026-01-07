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

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import from_seconds


def _build_config(
    *, apriltag_pos_var: float = 0.1, apriltag_yaw_var: float = 0.1
) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=1.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=5.0,
        dt_imu_max=1.0,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_sec=0.1,
        checkpoint_interval_sec=0.1,
        apriltag_pos_var=apriltag_pos_var,
        apriltag_yaw_var=apriltag_yaw_var,
        apriltag_gate_d2=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
    )


def _build_imu_sample(
    *, angular_velocity_rps: list[float], accel_body: list[float]
) -> ImuSample:
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=angular_velocity_rps,
        linear_acceleration_mps2=accel_body,
        angular_velocity_cov=gyro_cov,
        linear_acceleration_cov=accel_cov,
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


def test_yaw_wraps_positive_after_propagation() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    core._initialized = True
    core._x[8] = math.pi - 0.05

    imu_sample: ImuSample = _build_imu_sample(
        angular_velocity_rps=[0.0, 0.0, 1.0],
        accel_body=[0.0, 0.0, config.gravity_mps2],
    )
    start_time: EkfTime = from_seconds(0.0)
    next_time: EkfTime = from_seconds(0.1)

    start_event: EkfEvent = EkfEvent(
        t_meas=start_time,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )
    next_event: EkfEvent = EkfEvent(
        t_meas=next_time,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )

    core.process_event(start_event)
    core.process_event(next_event)

    yaw: float = float(core.state()[8])
    expected_raw: float = (math.pi - 0.05) + 0.1
    expected_wrapped: float = expected_raw - 2.0 * math.pi

    assert -math.pi <= yaw < math.pi
    assert math.isclose(yaw, expected_wrapped, abs_tol=1.0e-9)


def test_yaw_wraps_negative_after_measurement_update() -> None:
    config: EkfConfig = _build_config(apriltag_pos_var=0.0, apriltag_yaw_var=0.0)
    core: EkfCore = EkfCore(config)
    core._initialized = True
    core._x[8] = -math.pi + 0.02

    detection: AprilTagDetection = _build_apriltag_detection(
        pose_world_xyz_yaw=[0.0, 0.0, 0.0, -math.pi - 0.2]
    )
    apriltag_data: AprilTagDetectionArrayData = AprilTagDetectionArrayData(
        frame_id="camera",
        detections=[detection],
    )

    core.update_with_apriltags(apriltag_data, t_meas=0.0)

    yaw: float = float(core.state()[8])
    expected_wrapped: float = math.pi - 0.2

    assert -math.pi <= yaw < math.pi
    assert math.isclose(yaw, expected_wrapped, abs_tol=1.0e-9)
