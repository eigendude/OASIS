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

from types import ModuleType
from typing import Optional

import pytest


np: ModuleType = pytest.importorskip(
    "numpy", reason="TODO: requires numpy for EKF tests"
)

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import CameraInfoData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample


class _RecordingEkfCore(EkfCore):
    def __init__(self, config: EkfConfig) -> None:
        super().__init__(config)
        self.applied_events: list[EkfEventType] = []

    def process_event(self, event: EkfEvent) -> EkfOutputs:
        self.applied_events.append(event.event_type)
        return super().process_event(event)


def _build_config() -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=2.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=1.0,
        dt_imu_max=0.5,
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


def _build_apriltag_data() -> AprilTagDetectionArrayData:
    corners_px: list[float] = [0.0] * 8
    homography: list[float] = [0.0] * 9
    detection: AprilTagDetection = AprilTagDetection(
        family="tag36h11",
        tag_id=1,
        det_index_in_msg=0,
        corners_px=corners_px,
        pose_world_xyz_yaw=[0.0, 0.0, 0.0, 0.0],
        decision_margin=1.0,
        homography=homography,
    )
    return AprilTagDetectionArrayData(frame_id="camera", detections=[detection])


def test_replay_applies_events_in_priority_order() -> None:
    config: EkfConfig = _build_config()
    core: _RecordingEkfCore = _RecordingEkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)
    timestamp: EkfTime = EkfTime(sec=1, nanosec=0)

    apriltag_event: EkfEvent = EkfEvent(
        t_meas=timestamp,
        event_type=EkfEventType.APRILTAG,
        payload=_build_apriltag_data(),
    )
    mag_event: EkfEvent = EkfEvent(
        t_meas=timestamp,
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )
    imu_event: EkfEvent = EkfEvent(
        t_meas=timestamp,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )

    buffer.insert_event(apriltag_event)
    buffer.insert_event(mag_event)
    buffer.insert_event(imu_event)

    core.replay(buffer)

    assert core.applied_events == [
        EkfEventType.IMU,
        EkfEventType.MAG,
        EkfEventType.APRILTAG,
    ]


def test_mag_rejection_does_not_advance_frontier() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    imu_time: EkfTime = EkfTime(sec=0, nanosec=0)
    imu_event: EkfEvent = EkfEvent(
        t_meas=imu_time,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=_build_imu_sample(), calibration=None),
    )
    core.process_event(imu_event)

    mag_time: EkfTime = EkfTime(sec=1, nanosec=0)
    mag_cov: list[float] = [
        -1.0,
        0.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        -1.0,
    ]
    mag_sample: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=mag_cov,
    )
    mag_event: EkfEvent = EkfEvent(
        t_meas=mag_time,
        event_type=EkfEventType.MAG,
        payload=mag_sample,
    )
    output: EkfOutputs = core.process_event(mag_event)

    assert output.mag_update is not None
    assert not output.mag_update.accepted
    assert core.frontier_time() == imu_time


def test_imu_dt_over_max_advances_frontier() -> None:
    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    first_time: EkfTime = EkfTime(sec=0, nanosec=0)
    second_time: EkfTime = EkfTime(sec=1, nanosec=0)
    imu_sample: ImuSample = _build_imu_sample()

    core.process_event(
        EkfEvent(
            t_meas=first_time,
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )
    output: EkfOutputs = core.process_event(
        EkfEvent(
            t_meas=second_time,
            event_type=EkfEventType.IMU,
            payload=EkfImuPacket(imu=imu_sample, calibration=None),
        )
    )

    assert output.odom_time == second_time


def test_apriltag_linearization_uses_fixed_state() -> None:
    class RecordingAprilTagModel:
        def __init__(self) -> None:
            self.states: list[np.ndarray] = []

        def set_camera_info(self, camera_info: CameraInfoData) -> None:
            pass

        def pose_measurement(
            self, detection: AprilTagDetection
        ) -> Optional[np.ndarray]:
            return np.asarray(detection.pose_world_xyz_yaw, dtype=float)

        def linearize_pose(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            self.states.append(state.copy())
            z_hat: np.ndarray = np.asarray(
                [state[0], state[1], state[2], state[8]], dtype=float
            )
            h: np.ndarray = np.zeros((4, 9), dtype=float)
            h[0, 0] = 1.0
            h[1, 1] = 1.0
            h[2, 2] = 1.0
            h[3, 8] = 1.0
            return z_hat, h

    config: EkfConfig = _build_config()
    core: EkfCore = EkfCore(config)
    core._apriltag_model = RecordingAprilTagModel()
    core._camera_info = CameraInfoData(
        frame_id="camera",
        width=640,
        height=480,
        distortion_model="plumb_bob",
        d=[],
        k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    )

    detections: list[AprilTagDetection] = [
        AprilTagDetection(
            family="tag36h11",
            tag_id=1,
            det_index_in_msg=0,
            corners_px=[0.0] * 8,
            pose_world_xyz_yaw=[1.0, 0.0, 0.0, 0.0],
            decision_margin=1.0,
            homography=[0.0] * 9,
        ),
        AprilTagDetection(
            family="tag36h11",
            tag_id=2,
            det_index_in_msg=1,
            corners_px=[0.0] * 8,
            pose_world_xyz_yaw=[2.0, 0.0, 0.0, 0.0],
            decision_margin=1.0,
            homography=[0.0] * 9,
        ),
    ]
    apriltag_event: EkfEvent = EkfEvent(
        t_meas=EkfTime(sec=2, nanosec=0),
        event_type=EkfEventType.APRILTAG,
        payload=AprilTagDetectionArrayData(frame_id="camera", detections=detections),
    )

    core.process_event(apriltag_event)

    states: list[np.ndarray] = core._apriltag_model.states
    assert len(states) == 2
    assert np.allclose(states[0], states[1])
