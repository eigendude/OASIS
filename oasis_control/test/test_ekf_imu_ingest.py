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

from typing import Iterable
from typing import Optional

from oasis_control.localization.ekf.core.ekf_core import EkfCore
from oasis_control.localization.ekf.ekf_buffer import EkfBuffer
from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfEventType
from oasis_control.localization.ekf.ekf_types import EkfImuPacket
from oasis_control.localization.ekf.ekf_types import EkfOutputs
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import EkfUpdateData
from oasis_control.localization.ekf.ekf_types import ImuCalibrationData
from oasis_control.localization.ekf.ekf_types import ImuSample
from oasis_control.localization.ekf.ekf_types import MagSample
from oasis_control.localization.ekf.ekf_types import from_ns
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.imu_packet_validation import (
    imu_calibration_stamp_reject_reason,
)


# Nanoseconds per second for time conversions
_NS_PER_S: int = 1_000_000_000

# Nanoseconds per millisecond for time conversions
_NS_PER_MS: int = 1_000_000


def _ns_from_s(seconds: int) -> int:
    return seconds * _NS_PER_S


def _ns_from_ms(milliseconds: int) -> int:
    return milliseconds * _NS_PER_MS


def _build_config(*, dt_imu_max_ns: int) -> EkfConfig:
    return EkfConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_ns=_ns_from_s(1),
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
        checkpoint_interval_ns=_ns_from_ms(250),
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
        mag_alpha=0.01,
        mag_r_min_t2=[1.0e-12, 1.0e-12, 1.0e-12],
        mag_r_max_t2=[2.5e-9, 2.5e-9, 2.5e-9],
        mag_r0_t2=[4.0e-10, 4.0e-10, 4.0e-10],
        mag_world_t=[1.0, 0.0, 0.0],
    )


def _build_imu_sample(marker: float) -> ImuSample:
    accel_cov: list[float] = [0.0] * 9
    gyro_cov: list[float] = [0.0] * 9
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=[marker, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, 9.81],
        angular_velocity_cov=gyro_cov,
        linear_acceleration_cov=accel_cov,
    )


def _build_calibration(bias: float) -> ImuCalibrationData:
    accel_bias: list[float] = [bias, bias, bias]
    accel_a: list[float] = [
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
    accel_param_cov: list[float] = [0.0] * 144
    gyro_bias_cov: list[float] = [0.0] * 9
    return ImuCalibrationData(
        valid=True,
        frame_id="imu",
        accel_bias_mps2=accel_bias,
        accel_a=accel_a,
        accel_param_cov=accel_param_cov,
        gyro_bias_rps=accel_bias,
        gyro_bias_cov=gyro_bias_cov,
        gravity_mps2=9.81,
        fit_sample_count=100,
        rms_residual_mps2=0.0,
        temperature_c=20.0,
        temperature_var_c2=0.0,
    )


def _build_mag_sample() -> MagSample:
    mag_cov: list[float] = [0.0] * 9
    return MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 0.0, 0.0],
        magnetic_field_cov=mag_cov,
    )


def _build_imu_event(
    t_meas: EkfTime,
    imu_sample: ImuSample,
    calibration: Optional[ImuCalibrationData],
) -> EkfEvent:
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.IMU,
        payload=EkfImuPacket(imu=imu_sample, calibration=calibration),
    )


def _build_mag_event(t_meas: EkfTime) -> EkfEvent:
    return EkfEvent(
        t_meas=t_meas,
        event_type=EkfEventType.MAG,
        payload=_build_mag_sample(),
    )


def _collect_mag_updates(outputs: Iterable[EkfOutputs]) -> list[EkfUpdateData]:
    updates: list[EkfUpdateData] = []
    for output in outputs:
        if output.mag_update is not None:
            updates.append(output.mag_update)
    return updates


def _run_replay(events: list[EkfEvent], config: EkfConfig) -> list[EkfUpdateData]:
    buffer: EkfBuffer = EkfBuffer(config)
    core: EkfCore = EkfCore(config)
    for event in events:
        buffer.insert_event(event)
    outputs: list[EkfOutputs] = core.replay(buffer)
    return _collect_mag_updates(outputs)


def test_calibration_applied_once() -> None:
    config: EkfConfig = _build_config(dt_imu_max_ns=_ns_from_ms(50))
    core: EkfCore = EkfCore(config)

    first_cal: ImuCalibrationData = _build_calibration(0.1)
    second_cal: ImuCalibrationData = _build_calibration(0.5)
    imu_sample: ImuSample = _build_imu_sample(0.0)

    core.process_event(_build_imu_event(from_ns(0), imu_sample, first_cal))
    core.process_event(
        _build_imu_event(from_ns(_ns_from_ms(10)), imu_sample, second_cal)
    )

    state = core._state
    assert state.imu_bias_accel_mps2.tolist() == first_cal.accel_bias_mps2
    assert state.imu_bias_gyro_rps.tolist() == first_cal.gyro_bias_rps


def test_calibration_stamp_mismatch_rejects() -> None:
    imu_stamp: EkfTime = EkfTime(sec=1, nanosec=0)
    cal_stamp: EkfTime = EkfTime(sec=1, nanosec=1)
    reason: Optional[str] = imu_calibration_stamp_reject_reason(imu_stamp, cal_stamp)
    assert reason == "imu_calibration_stamp_mismatch"
    assert imu_calibration_stamp_reject_reason(imu_stamp, imu_stamp) is None


def test_gap_rejects_replay() -> None:
    config: EkfConfig = _build_config(dt_imu_max_ns=_ns_from_ms(50))
    events: list[EkfEvent] = [
        _build_imu_event(from_ns(0), _build_imu_sample(0.0), None),
        _build_imu_event(from_ns(_ns_from_ms(10)), _build_imu_sample(0.1), None),
        _build_mag_event(from_ns(_ns_from_ms(200))),
        _build_imu_event(from_ns(_ns_from_ms(500)), _build_imu_sample(0.2), None),
    ]

    updates: list[EkfUpdateData] = _run_replay(events, config)

    assert len(updates) == 1
    assert not updates[0].accepted
    assert updates[0].reject_reason == "imu_gap"


def test_out_of_order_imu_does_not_overwrite_last_sample() -> None:
    config: EkfConfig = _build_config(dt_imu_max_ns=_ns_from_ms(50))
    core: EkfCore = EkfCore(config)
    imu_sample_early: ImuSample = _build_imu_sample(0.1)
    imu_sample_late: ImuSample = _build_imu_sample(0.2)
    imu_sample_out_of_order: ImuSample = _build_imu_sample(0.3)

    core.process_event(_build_imu_event(from_ns(0), imu_sample_early, None))
    core.process_event(
        _build_imu_event(from_ns(_ns_from_ms(10)), imu_sample_late, None)
    )
    core.process_event(
        _build_imu_event(from_ns(_ns_from_ms(5)), imu_sample_out_of_order, None)
    )

    assert core._last_imu_time_ns == _ns_from_ms(10)
    assert core._last_imu is not None
    assert core._last_imu.angular_velocity_rps == imu_sample_late.angular_velocity_rps
    assert core._imu_times_ns == [0, _ns_from_ms(5), _ns_from_ms(10)]
    frontier_time: Optional[EkfTime] = core.frontier_time()
    assert frontier_time is not None
    assert to_ns(frontier_time) == _ns_from_ms(10)
