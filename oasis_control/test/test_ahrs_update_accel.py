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

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_filter import AhrsFilter
from oasis_control.localization.ahrs.ahrs_quat import quat_from_rotvec_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsImuPacket
from oasis_control.localization.ahrs.ahrs_types import AhrsOutputs
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import ImuCalibrationData
from oasis_control.localization.ahrs.ahrs_types import ImuSample
from oasis_control.localization.ahrs.ahrs_update_accel import update_accel


class FixedClock(AhrsClock):
    def __init__(self, now_sec: int, now_nanosec: int) -> None:
        self._now_sec: int = now_sec
        self._now_nanosec: int = now_nanosec

    def now(self) -> AhrsTime:
        return AhrsTime(sec=self._now_sec, nanosec=self._now_nanosec)


def _config(
    *,
    accel_gate_d2_threshold: float,
    accel_use_direction_only: bool = False,
) -> AhrsConfig:
    return AhrsConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=0.0,
        epsilon_wall_future_sec=0.0,
        dt_clock_jump_max_sec=0.0,
        dt_imu_max_sec=0.0,
        gyro_gate_d2_threshold=9.0,
        accel_gate_d2_threshold=accel_gate_d2_threshold,
        accel_use_direction_only=accel_use_direction_only,
        mag_alpha=1.0,
        mag_r_min=[0.0] * 9,
        mag_r_max=[0.0] * 9,
        mag_r0=[0.0] * 9,
        q_v=0.0,
        q_w=0.0,
        q_bg=0.0,
        q_ba=0.0,
        q_a=0.0,
        q_bi=0.0,
        q_bm=0.0,
        q_g=0.0,
        q_m=0.0,
    )


def _cov_diag(value: float) -> list[float]:
    return [
        value,
        0.0,
        0.0,
        0.0,
        value,
        0.0,
        0.0,
        0.0,
        value,
    ]


def _initial_covariance(layout: AhrsErrorStateLayout, value: float) -> list[float]:
    dim: int = layout.dim
    p: list[float] = [0.0] * (dim * dim)
    for i in range(dim):
        p[i * dim + i] = value
    return p


def test_accel_update_accepts_and_leaves_attitude_when_consistent() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.g_w_mps2 = [0.0, 0.0, -9.81]
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, -9.81],
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_accel(
        layout,
        state,
        p,
        imu,
        _config(accel_gate_d2_threshold=9.0),
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is True
    assert update_report.reject_reason is None
    assert all(
        math.isclose(value, 0.0, rel_tol=0.0, abs_tol=1.0e-9)
        for value in update_report.nu
    )
    assert updated_state.q_wb_wxyz == state.q_wb_wxyz
    assert updated_p != p


def test_accel_update_adjusts_attitude_for_misaligned_gravity() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.g_w_mps2 = [0.0, 0.0, -9.81]
    state.q_wb_wxyz = quat_from_rotvec_wxyz([0.1, 0.0, 0.0])
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, -9.81],
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_accel(
        layout,
        state,
        p,
        imu,
        _config(accel_gate_d2_threshold=9.0),
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is True
    quat_delta: float = sum(
        abs(a - b)
        for a, b in zip(updated_state.q_wb_wxyz, state.q_wb_wxyz, strict=True)
    )
    assert quat_delta > 1.0e-6
    norm: float = math.sqrt(sum(value * value for value in updated_state.q_wb_wxyz))
    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert updated_p != p


def test_accel_update_rejects_large_innovation() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.g_w_mps2 = [0.0, 0.0, -9.81]
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[10.0, 0.0, 0.0],
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_accel(
        layout,
        state,
        p,
        imu,
        _config(accel_gate_d2_threshold=0.1),
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is False
    assert update_report.reject_reason == "mahalanobis_gate"
    assert updated_state == state
    assert updated_p == p


def test_accel_update_rejects_uninitialized_gravity() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.g_w_mps2 = [0.0, 0.0, 0.0]
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, -9.81],
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_accel(
        layout,
        state,
        p,
        imu,
        _config(accel_gate_d2_threshold=9.0),
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is False
    assert update_report.reject_reason == "uninitialized_gravity"
    assert updated_state == state
    assert updated_p == p


def test_accel_update_direction_only_accepts_magnitude_mismatch() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.g_w_mps2 = [0.0, 0.0, -9.81]
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, -19.62],
        angular_velocity_cov=[0.0] * 9,
        linear_acceleration_cov=_cov_diag(0.01),
    )

    config: AhrsConfig = _config(
        accel_gate_d2_threshold=9.0,
        accel_use_direction_only=True,
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_accel(
        layout,
        state,
        p,
        imu,
        config,
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is True
    assert all(
        math.isclose(value, 0.0, rel_tol=0.0, abs_tol=1.0e-6)
        for value in update_report.nu
    )
    assert updated_state.q_wb_wxyz == state.q_wb_wxyz
    assert updated_p != p


def _imu_event_at(sec: int, nanosec: int) -> AhrsEvent:
    imu: ImuSample = ImuSample(
        frame_id="imu",
        angular_velocity_rps=[0.0, 0.0, 0.0],
        linear_acceleration_mps2=[0.0, 0.0, 0.0],
        angular_velocity_cov=_cov_diag(0.01),
        linear_acceleration_cov=_cov_diag(0.01),
    )
    calibration: ImuCalibrationData = ImuCalibrationData(
        valid=True,
        frame_id="imu",
        accel_bias_mps2=[0.0, 0.0, 0.0],
        accel_a=[0.0] * 9,
        accel_param_cov=[0.0] * 144,
        gyro_bias_rps=[0.0, 0.0, 0.0],
        gyro_bias_cov=[0.0] * 9,
        gravity_mps2=9.81,
        fit_sample_count=0,
        rms_residual_mps2=0.0,
        temperature_c=20.0,
        temperature_var_c2=0.0,
    )
    packet: AhrsImuPacket = AhrsImuPacket(imu=imu, calibration=calibration)

    return AhrsEvent(
        t_meas=AhrsTime(sec=sec, nanosec=nanosec),
        topic="imu",
        frame_id="imu",
        event_type=AhrsEventType.IMU,
        payload=packet,
    )


def test_filter_reports_accel_update() -> None:
    clock: FixedClock = FixedClock(now_sec=10, now_nanosec=0)
    filt: AhrsFilter = AhrsFilter(
        config=_config(accel_gate_d2_threshold=9.0),
        clock=clock,
    )

    outputs: AhrsOutputs = filt.handle_event(_imu_event_at(1, 0))

    accel_update: AhrsUpdateData
    assert outputs.accel_update is not None
    accel_update = outputs.accel_update
    assert accel_update.accepted is False
    assert accel_update.reject_reason == "uninitialized_gravity"
    assert accel_update.z == [0.0, 0.0, 0.0]
    assert accel_update.r.rows == 3
    assert accel_update.s.rows == 0
