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
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_seconds


def _build_config(
    *,
    dt_imu_max: float,
    checkpoint_interval_sec: float,
    apriltag_gate_d2: float,
    apriltag_pos_var: float,
    apriltag_yaw_var: float,
) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=2.0,
        epsilon_wall_future=0.1,
        dt_clock_jump_max=5.0,
        dt_imu_max=dt_imu_max,
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
    )


def _build_imu_sample(
    *, accel_body: list[float], angular_velocity: Optional[list[float]] = None
) -> ImuSample:
    angular_velocity_rps: list[float] = (
        [0.0, 0.0, 0.0] if angular_velocity is None else angular_velocity
    )
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=angular_velocity_rps,
        linear_acceleration_mps2=accel_body,
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


def test_checkpoint_inclusive_time() -> None:
    config: EkfConfig = _build_config(
        dt_imu_max=0.5,
        checkpoint_interval_sec=0.2,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    core: EkfCore = EkfCore(config)
    buffer: EkfBuffer = EkfBuffer(config)
    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )

    event_first: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )
    buffer.insert_event(event_first)
    core.process_event(event_first)

    event_second: EkfEvent = EkfEvent(
        t_meas=from_seconds(1.5),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )
    buffer.insert_event(event_second)
    core.process_event(event_second)

    checkpoint: Optional[tuple[float, np.ndarray, np.ndarray]] = (
        core._checkpoint_snapshot(from_seconds(1.5))
    )
    assert checkpoint is not None
    checkpoint_time: float
    checkpoint_state: np.ndarray
    checkpoint_cov: np.ndarray
    checkpoint_time, checkpoint_state, checkpoint_cov = checkpoint

    outputs: EkfOutputs = core.replay(buffer, start_time=from_seconds(1.5))
    assert outputs is not None
    assert core.frontier_time() == checkpoint_time
    assert np.allclose(core.state(), checkpoint_state, atol=1.0e-12)
    assert np.allclose(core.covariance(), checkpoint_cov, atol=1.0e-12)


def test_dt_imu_max_gates_dt_total_not_imu_age() -> None:
    config: EkfConfig = _build_config(
        dt_imu_max=0.05,
        checkpoint_interval_sec=1.0,
        apriltag_gate_d2=0.0,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    core: EkfCore = EkfCore(config)
    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[1.0, 0.0, config.gravity_mps2]
    )

    imu_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )
    core.process_event(imu_event)
    state_initial: np.ndarray = core.state()

    mag_event_skip: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.5),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )
    core.process_event(mag_event_skip)
    state_after_skip: np.ndarray = core.state()
    assert np.allclose(state_after_skip, state_initial, atol=1.0e-12)

    mag_event_step: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.51),
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )
    core.process_event(mag_event_step)
    state_after_step: np.ndarray = core.state()
    assert state_after_step[0] > 0.0
    assert state_after_step[3] > 0.0


def test_apriltag_mahalanobis_rejection_does_not_update_state() -> None:
    config: EkfConfig = _build_config(
        dt_imu_max=0.5,
        checkpoint_interval_sec=1.0,
        apriltag_gate_d2=0.5,
        apriltag_pos_var=1.0,
        apriltag_yaw_var=1.0,
    )
    core: EkfCore = EkfCore(config)
    imu_sample: ImuSample = _build_imu_sample(
        accel_body=[0.0, 0.0, config.gravity_mps2]
    )
    imu_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.0),
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=None),
    )
    core.process_event(imu_event)
    state_before: np.ndarray = core.state()
    cov_before: np.ndarray = core.covariance()

    far_detection: AprilTagDetection = _build_apriltag_detection(
        pose_world_xyz_yaw=[10.0, 0.0, 0.0, 0.0],
        tag_id=1,
    )
    far_data: AprilTagDetectionArrayData = AprilTagDetectionArrayData(
        frame_id="camera",
        detections=[far_detection],
    )
    far_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.01),
        event_type=EkfEventType.APRILTAG,
        payload=far_data,
    )
    far_outputs: EkfOutputs = core.process_event(far_event)
    far_update_data: Optional[EkfUpdateData] = None
    assert far_outputs.apriltag_update is not None
    far_update_data = far_outputs.apriltag_update.detections[0].update

    assert far_update_data.accepted is False
    assert far_update_data.reject_reason == "mahalanobis_gate"
    assert far_update_data.maha_d2 > config.apriltag_gate_d2
    assert np.allclose(core.state(), state_before, atol=1.0e-12)
    assert np.allclose(core.covariance(), cov_before, atol=1.0e-12)

    near_detection: AprilTagDetection = _build_apriltag_detection(
        pose_world_xyz_yaw=[0.01, 0.0, 0.0, 0.0],
        tag_id=2,
    )
    near_data: AprilTagDetectionArrayData = AprilTagDetectionArrayData(
        frame_id="camera",
        detections=[near_detection],
    )
    near_event: EkfEvent = EkfEvent(
        t_meas=from_seconds(0.02),
        event_type=EkfEventType.APRILTAG,
        payload=near_data,
    )
    near_outputs: EkfOutputs = core.process_event(near_event)
    assert near_outputs.apriltag_update is not None
    near_update_data: EkfUpdateData = near_outputs.apriltag_update.detections[0].update
    assert near_update_data.accepted is True
    assert not np.allclose(core.state(), state_before, atol=1.0e-12)
