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

from typing import Any

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
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
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.models.apriltag_measurement_model import (
    AprilTagMeasurementModel,
)


def _build_config(*, dt_imu_max: float = 0.5) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=2.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=1.0,
        dt_imu_max=dt_imu_max,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=1.0,
        accel_noise_var=0.1,
        gyro_noise_var=0.1,
        gravity_mps2=9.81,
        max_dt_sec=0.01,
        checkpoint_interval_sec=0.5,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
        apriltag_gate_d2=0.0,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=0.1,
        tag_landmark_prior_sigma_rot_rad=0.1,
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
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    core.process_event(imu_event)
    frontier_before: EkfTime | None = core.frontier_time()
    assert frontier_before is not None

    mag_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.0),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )
    outputs: EkfOutputs = core.process_event(mag_event)

    assert outputs.odom_time is None
    assert outputs.mag_update is not None
    assert not outputs.mag_update.accepted
    assert core.frontier_time() == frontier_before


def test_imu_max_dt_advances_frontier() -> None:
    config: EkfConfig = _build_config(dt_imu_max=0.1)
    core: EkfCore = EkfCore(config)

    imu_event_0: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    core.process_event(imu_event_0)

    imu_event_1: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    outputs: EkfOutputs = core.process_event(imu_event_1)

    assert outputs.odom_time == imu_event_1.t_meas
    assert core.frontier_time() == imu_event_1.t_meas


def test_apriltag_linearization_uses_fixed_state() -> None:
    config: EkfConfig = _build_config()
    core: _RecordingAprilTagEkf = _RecordingAprilTagEkf(config)

    camera_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.CAMERA_INFO,
        payload=_build_camera_info(),
    )
    core.process_event(camera_event)

    detections: list[AprilTagDetection] = [
        AprilTagDetection(
            family="tag36h11",
            tag_id=1,
            det_index_in_msg=0,
            corners_px=[0.0] * 8,
            pose_world_xyz_yaw=[1.0, 0.0, 0.0, 0.1],
            decision_margin=1.0,
            homography=[0.0] * 9,
        ),
        AprilTagDetection(
            family="tag36h11",
            tag_id=2,
            det_index_in_msg=1,
            corners_px=[0.0] * 8,
            pose_world_xyz_yaw=[2.0, 0.5, 0.0, -0.2],
            decision_margin=1.0,
            homography=[0.0] * 9,
        ),
    ]
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.0),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(frame_id="camera", detections=detections),
    )
    initial_state: list[float] = core.state().tolist()
    core.process_event(apriltag_event)

    model: _RecordingAprilTagModel = core._apriltag_model
    assert len(model.linearize_inputs) == 2
    assert model.linearize_inputs[0] == initial_state
    assert model.linearize_inputs[1] == initial_state
    assert core.state().tolist() != initial_state
