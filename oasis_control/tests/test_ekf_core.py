import math

import numpy as np
import pytest

from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_types import AprilTagDetection
from oasis_control.localization.ekf.ekf_types import AprilTagDetectionArrayData
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import ImuSample


def _build_config(t_buffer_sec: float = 2.0) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=t_buffer_sec,
        epsilon_wall_future=0.05,
        dt_clock_jump_max=1.0,
        dt_imu_max=0.1,
        pos_var=1.0,
        vel_var=1.0,
        ang_var=0.25,
        accel_noise_var=0.01,
        gyro_noise_var=0.001,
        gravity_mps2=9.81,
        max_dt_sec=0.05,
        checkpoint_interval_sec=0.5,
        apriltag_pos_var=0.1,
        apriltag_yaw_var=0.1,
        tag_size_m=0.16,
        tag_anchor_family="tag36h11",
        tag_anchor_id=0,
        tag_landmark_prior_sigma_t_m=10.0,
        tag_landmark_prior_sigma_rot_rad=math.pi,
    )


def _process_event(buffer: EkfBuffer, core: EkfCore, event: EkfEvent) -> None:
    buffer.insert_event(event)
    buffer.evict(event.t_meas)
    if core.is_out_of_order(event.t_meas):
        core.replay(buffer, event.t_meas)
    else:
        core.process_event(event)


def _imu_event(t_meas: float, accel_mps2: list[float]) -> EkfEvent:
    sample: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=accel_mps2,
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=[0.0] * 9,
    )
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=sample, calibration=None),
    )


def _apriltag_event(t_meas: float, pose_xyz_yaw: list[float]) -> EkfEvent:
    detection: AprilTagDetection = AprilTagDetection(
        family="tag36h11",
        tag_id=0,
        det_index_in_msg=0,
        corners_px=[],
        pose_world_xyz_yaw=pose_xyz_yaw,
        decision_margin=1.0,
        homography=[0.0] * 9,
    )
    data: AprilTagDetectionArrayData = AprilTagDetectionArrayData(
        frame_id="camera",
        detections=[detection],
    )
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.APRILTAG,
        payload=data,
    )


def test_in_order_imu_propagation() -> None:
    config: EkfConfig = _build_config()
    buffer: EkfBuffer = EkfBuffer(config)
    core: EkfCore = EkfCore(config)

    _process_event(buffer, core, _imu_event(0.0, [1.0, 0.0, 9.81]))
    _process_event(buffer, core, _imu_event(0.1, [1.0, 0.0, 9.81]))

    state: np.ndarray = core.state()
    assert core.frontier_time() == 0.1
    assert state[3] == pytest.approx(0.1, abs=1.0e-6)
    assert state[0] == pytest.approx(0.01, abs=1.0e-6)


def test_out_of_order_apriltag_replay_matches_in_order() -> None:
    config: EkfConfig = _build_config()

    in_order_buffer: EkfBuffer = EkfBuffer(config)
    in_order_core: EkfCore = EkfCore(config)
    _process_event(in_order_buffer, in_order_core, _imu_event(0.0, [0.0, 0.0, 9.81]))
    _process_event(in_order_buffer, in_order_core, _apriltag_event(0.5, [1.0, 0.0, 0.0, 0.0]))
    _process_event(in_order_buffer, in_order_core, _imu_event(1.0, [0.0, 0.0, 9.81]))

    out_of_order_buffer: EkfBuffer = EkfBuffer(config)
    out_of_order_core: EkfCore = EkfCore(config)
    _process_event(out_of_order_buffer, out_of_order_core, _imu_event(0.0, [0.0, 0.0, 9.81]))
    _process_event(out_of_order_buffer, out_of_order_core, _imu_event(1.0, [0.0, 0.0, 9.81]))
    _process_event(out_of_order_buffer, out_of_order_core, _apriltag_event(0.5, [1.0, 0.0, 0.0, 0.0]))

    assert np.allclose(in_order_core.state(), out_of_order_core.state(), atol=1.0e-6)


def test_eviction_keeps_latest_time_and_replay_within_lag() -> None:
    config: EkfConfig = _build_config(t_buffer_sec=1.0)

    in_order_buffer: EkfBuffer = EkfBuffer(config)
    in_order_core: EkfCore = EkfCore(config)
    _process_event(in_order_buffer, in_order_core, _imu_event(0.0, [0.0, 0.0, 9.81]))
    _process_event(in_order_buffer, in_order_core, _apriltag_event(0.5, [0.5, 0.0, 0.0, 0.0]))
    _process_event(in_order_buffer, in_order_core, _apriltag_event(1.2, [1.0, 0.0, 0.0, 0.0]))
    _process_event(in_order_buffer, in_order_core, _imu_event(1.5, [0.0, 0.0, 9.81]))

    out_of_order_buffer: EkfBuffer = EkfBuffer(config)
    out_of_order_core: EkfCore = EkfCore(config)
    _process_event(out_of_order_buffer, out_of_order_core, _imu_event(0.0, [0.0, 0.0, 9.81]))
    _process_event(out_of_order_buffer, out_of_order_core, _apriltag_event(0.5, [0.5, 0.0, 0.0, 0.0]))
    _process_event(out_of_order_buffer, out_of_order_core, _imu_event(1.5, [0.0, 0.0, 9.81]))

    latest_time: float = float(out_of_order_buffer.latest_time())
    assert latest_time == pytest.approx(1.5)

    _process_event(out_of_order_buffer, out_of_order_core, _apriltag_event(1.2, [1.0, 0.0, 0.0, 0.0]))

    assert np.allclose(in_order_core.state(), out_of_order_core.state(), atol=1.0e-6)
