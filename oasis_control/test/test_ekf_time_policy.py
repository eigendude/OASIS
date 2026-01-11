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
from typing import Optional
from typing import cast

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_state import Pose3
from oasis_control.localization.ekf.ekf_state import TagKey
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfMatrix
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_ns
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)


_NS_PER_S: int = 1_000_000_000
_NS_PER_MS: int = 1_000_000


def _ns_from_s(seconds: int) -> int:
    return seconds * _NS_PER_S


def _ns_from_ms(milliseconds: int) -> int:
    return milliseconds * _NS_PER_MS


def _build_config(*, dt_imu_max_ns: int = _ns_from_ms(500)) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=_ns_from_s(2),
        epsilon_wall_future_ns=_ns_from_ms(100),
        dt_clock_jump_max_ns=_ns_from_s(1),
        dt_imu_max_ns=dt_imu_max_ns,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_ns=_ns_from_ms(10),
        checkpoint_interval_ns=_ns_from_ms(500),
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


def _build_mag_sample() -> MagSample:
    mag_cov: list[float] = [0.0] * 9
    return MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=mag_cov,
    )


def _build_camera_info() -> CameraInfoData:
    return CameraInfoData(
        frame_id="camera",
        width=640,
        height=480,
        distortion_model="plumb_bob",
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
        r=[
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
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


def test_landmark_augmentation_expands_covariance() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    core._ensure_initialized()
    initial_dim: int = int(core.covariance().shape[0])

    tag_key: TagKey = TagKey(family="tag36h11", tag_id=5)
    added: bool = core._state.ensure_landmark(tag_key)
    assert added

    updated_cov: Any = core.covariance()
    updated_dim: int = int(updated_cov.shape[0])
    assert updated_dim == initial_dim + 6

    landmark_slice: Optional[slice] = core._state.landmark_slice(tag_key)
    assert landmark_slice is not None

    sigma_t: float = config.tag_landmark_prior_sigma_t_m
    sigma_rot: float = config.tag_landmark_prior_sigma_rot_rad
    start: int = landmark_slice.start
    value: float
    for offset in range(3):
        value = float(updated_cov[start + offset, start + offset])
        assert math.isclose(value, sigma_t**2, rel_tol=0.0, abs_tol=1.0e-12)
    for offset in range(3, 6):
        value = float(updated_cov[start + offset, start + offset])
        assert math.isclose(value, sigma_rot**2, rel_tol=0.0, abs_tol=1.0e-12)


class _RejectingMagEkf(EkfCore):
    def update_with_mag(self, mag_sample: MagSample, t_meas: EkfTime) -> EkfUpdateData:
        z_dim: int = 3
        zero_matrix: EkfMatrix = EkfMatrix(rows=z_dim, cols=z_dim, data=[0.0] * 9)
        return EkfUpdateData(
            sensor="magnetic_field",
            frame_id=mag_sample.frame_id,
            t_meas=t_meas,
            accepted=False,
            reject_reason="forced",
            z_dim=z_dim,
            z=[0.0, 0.0, 0.0],
            z_hat=[0.0, 0.0, 0.0],
            nu=[0.0, 0.0, 0.0],
            r=zero_matrix,
            s_hat=zero_matrix,
            s=zero_matrix,
            maha_d2=0.0,
            gate_d2_threshold=0.0,
            reproj_rms_px=0.0,
        )


class _RecordingAprilTagModel(AprilTagMeasurementModel):
    def __init__(self) -> None:
        super().__init__()
        self.linearize_inputs: list[list[float]] = []

    def linearize_pose(self, state: Any) -> tuple[Any, Any]:
        self.linearize_inputs.append(list(state.tolist()))
        return super().linearize_pose(state)


class _RecordingAprilTagEkf(EkfCore):
    def __init__(self, config: EkfConfig) -> None:
        super().__init__(config)
        self._apriltag_model = _RecordingAprilTagModel()


def test_mag_rejection_does_not_advance_frontier() -> None:
    config: EkfConfig = _build_config()
    core: _RejectingMagEkf = _RejectingMagEkf(config)

    imu_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    core.process_event(imu_event)
    frontier_before: EkfTime | None = core.frontier_time()
    assert frontier_before is not None

    mag_event: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_s(1)),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )
    outputs: EkfOutputs = core.process_event(mag_event)

    assert outputs.odom_time is None
    assert outputs.mag_update is not None
    assert not outputs.mag_update.accepted
    assert core.frontier_time() == frontier_before


def test_imu_max_dt_advances_frontier() -> None:
    config: EkfConfig = _build_config(dt_imu_max_ns=_ns_from_ms(100))
    core: EkfCore = EkfCore(config)

    imu_event_0: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    core.process_event(imu_event_0)

    imu_event_1: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_s(1)),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    outputs: EkfOutputs = core.process_event(imu_event_1)

    assert outputs.odom_time == imu_event_1.t_meas
    assert core.frontier_time() == imu_event_1.t_meas


def test_apriltag_linearization_tracks_state_updates() -> None:
    config: EkfConfig = _build_config()
    core: _RecordingAprilTagEkf = _RecordingAprilTagEkf(config)

    camera_info: CameraInfoData = _build_camera_info()
    camera_event: EkfEvent = EkfEvent(
        t_meas=from_ns(0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=camera_info,
    )
    core.process_event(camera_event)

    detections: list[AprilTagDetection] = [
        AprilTagDetection(
            family="tag36h11",
            tag_id=1,
            det_index_in_msg=0,
            corners_px=_project_tag_corners(
                pose_cam_xyz_yaw=[1.0, 0.0, 1.0, 0.1],
                tag_size_m=config.tag_size_m,
                camera_info=camera_info,
            ),
            pose_cam_xyz_yaw=[1.0, 0.0, 1.0, 0.1],
            decision_margin=1.0,
            homography=[0.0] * 9,
        ),
        AprilTagDetection(
            family="tag36h11",
            tag_id=2,
            det_index_in_msg=1,
            corners_px=_project_tag_corners(
                pose_cam_xyz_yaw=[2.0, 0.5, 1.0, -0.2],
                tag_size_m=config.tag_size_m,
                camera_info=camera_info,
            ),
            pose_cam_xyz_yaw=[2.0, 0.5, 1.0, -0.2],
            decision_margin=1.0,
            homography=[0.0] * 9,
        ),
    ]
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=from_ns(_ns_from_s(1)),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(frame_id="camera", detections=detections),
    )
    initial_state: list[float] = core.state().tolist()
    core.process_event(apriltag_event)

    model: _RecordingAprilTagModel = cast(_RecordingAprilTagModel, core._apriltag_model)
    assert len(model.linearize_inputs) == 2
    assert model.linearize_inputs[0] == initial_state
    assert model.linearize_inputs[1] != initial_state
    assert core.state().tolist() == initial_state
    world_odom: Pose3 = core.world_odom_pose()
    assert abs(float(world_odom.translation_m[0])) > 0.0
