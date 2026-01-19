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

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_types import ImuSample
from oasis_control.localization.ahrs.ahrs_update_gyro import update_gyro


def _config(*, gyro_gate_d2_threshold: float) -> AhrsConfig:
    return AhrsConfig(
        world_frame_id="world",
        odom_frame_id="odom",
        body_frame_id="base_link",
        t_buffer_sec=0.0,
        epsilon_wall_future_sec=0.0,
        dt_clock_jump_max_sec=0.0,
        dt_imu_max_sec=0.0,
        gyro_gate_d2_threshold=gyro_gate_d2_threshold,
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


def _imu_sample(omega_rps: list[float], cov: list[float]) -> ImuSample:
    return ImuSample(
        frame_id="imu",
        angular_velocity_rps=list(omega_rps),
        linear_acceleration_mps2=[0.0, 0.0, 0.0],
        angular_velocity_cov=list(cov),
        linear_acceleration_cov=[0.0] * 9,
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


def test_gyro_update_accepts_and_updates_omega() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = _imu_sample([1.0, 0.0, 0.0], _cov_diag(0.01))

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_gyro(
        layout,
        state,
        p,
        imu,
        _config(gyro_gate_d2_threshold=10.0),
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    expected_s: float = 0.1 + 0.1 + 0.01
    expected_gain_omega: float = 0.1 / expected_s
    expected_gain_bg: float = 0.1 / expected_s
    expected_omega: float = expected_gain_omega
    expected_bg: float = expected_gain_bg
    assert update_report.accepted is True
    assert update_report.reject_reason is None
    assert math.isclose(
        updated_state.omega_wb_rps[0],
        expected_omega,
        rel_tol=0.0,
        abs_tol=1.0e-6,
    )
    assert math.isclose(
        updated_state.b_g_rps[0],
        expected_bg,
        rel_tol=0.0,
        abs_tol=1.0e-6,
    )
    sl_omega: slice = layout.sl_omega()
    omega_diag_index: int = sl_omega.start * layout.dim + sl_omega.start
    assert updated_p[omega_diag_index] < 0.1


def test_gyro_update_rejects_large_innovation() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    p: list[float] = _initial_covariance(layout, value=0.1)
    imu: ImuSample = _imu_sample([10.0, 0.0, 0.0], _cov_diag(0.01))

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report = update_gyro(
        layout,
        state,
        p,
        imu,
        _config(gyro_gate_d2_threshold=1.0),
        frame_id="imu",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is False
    assert update_report.reject_reason == "mahalanobis_gate"
    assert updated_state.omega_wb_rps == state.omega_wb_rps
    assert updated_p == p
