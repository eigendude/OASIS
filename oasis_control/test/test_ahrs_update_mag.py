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
from oasis_control.localization.ahrs.ahrs_types import MagSample
from oasis_control.localization.ahrs.ahrs_update_mag import update_mag


def _config(
    *,
    mag_gate_d2_threshold: float,
    mag_use_direction_only: bool = False,
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
        accel_gate_d2_threshold=9.0,
        accel_use_direction_only=False,
        mag_gate_d2_threshold=mag_gate_d2_threshold,
        mag_use_direction_only=mag_use_direction_only,
        mag_alpha=0.0,
        mag_r_min=_cov_diag(0.0),
        mag_r_max=_cov_diag(10.0),
        mag_r0=_cov_diag(0.01),
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


def test_mag_update_accepts_consistent_measurement() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.m_w_t = [0.2, -0.1, 0.05]
    p: list[float] = _initial_covariance(layout, value=0.1)
    mag: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.2, -0.1, 0.05],
        magnetic_field_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report, _ = update_mag(
        layout,
        state,
        p,
        mag,
        _config(mag_gate_d2_threshold=9.0),
        frame_id="mag",
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


def test_mag_update_rejects_large_innovation() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.m_w_t = [1.0, 0.0, 0.0]
    p: list[float] = _initial_covariance(layout, value=0.1)
    mag: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.0, 1.0, 0.0],
        magnetic_field_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report, _ = update_mag(
        layout,
        state,
        p,
        mag,
        _config(mag_gate_d2_threshold=0.1),
        frame_id="mag",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is False
    assert update_report.reject_reason == "mahalanobis_gate"
    assert updated_state == state
    assert updated_p == p


def test_mag_update_direction_only_handles_magnitude_mismatch() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.m_w_t = [1.0, 0.0, 0.0]
    p: list[float] = _initial_covariance(layout, value=0.1)
    mag: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[10.0, 0.0, 0.0],
        magnetic_field_cov=_cov_diag(0.01),
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report, _ = update_mag(
        layout,
        state,
        p,
        mag,
        _config(mag_gate_d2_threshold=9.0, mag_use_direction_only=True),
        frame_id="mag",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is True
    assert all(
        math.isclose(value, 0.0, rel_tol=0.0, abs_tol=1.0e-9)
        for value in update_report.nu
    )
    assert updated_state.q_wb_wxyz == state.q_wb_wxyz
    assert updated_p != p


def test_mag_update_rejects_uninitialized_field_and_preserves_covariance() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.m_w_t = [0.0, 0.0, 0.0]
    p: list[float] = _initial_covariance(layout, value=0.1)
    r_raw: list[float] = _cov_diag(0.02)
    mag: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.1, 0.0, -0.2],
        magnetic_field_cov=r_raw,
    )

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    r_next: list[float]
    updated_state, updated_p, update_report, r_next = update_mag(
        layout,
        state,
        p,
        mag,
        _config(mag_gate_d2_threshold=9.0),
        frame_id="mag",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is False
    assert update_report.reject_reason == "uninitialized_magnetic_field"
    assert updated_state == state
    assert updated_p == p
    assert update_report.r.data == r_raw
    assert r_next[0] > 0.0
    assert r_next[4] > 0.0
    assert r_next[8] > 0.0
    assert math.isclose(r_next[1], r_next[3], rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(r_next[2], r_next[6], rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(r_next[5], r_next[7], rel_tol=0.0, abs_tol=1.0e-9)


def test_mag_update_prefers_r_state_over_raw_covariance() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.m_w_t = [0.3, 0.0, 0.0]
    p: list[float] = _initial_covariance(layout, value=0.1)
    mag: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.3, 0.0, 0.0],
        magnetic_field_cov=_cov_diag(9.999),
    )
    r_state: list[float] = _cov_diag(0.123)

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report, _ = update_mag(
        layout,
        state,
        p,
        mag,
        _config(mag_gate_d2_threshold=9.0, mag_use_direction_only=True),
        frame_id="mag",
        t_meas=AhrsTime(sec=1, nanosec=0),
        r_state=r_state,
    )

    r_scale: float = 1.0 / (0.3 * 0.3)
    expected_r: list[float] = [value * r_scale for value in r_state]
    assert update_report.accepted is True
    assert updated_state.q_wb_wxyz == state.q_wb_wxyz
    assert updated_p != p
    assert all(
        math.isclose(value, expected_r[index], rel_tol=0.0, abs_tol=1.0e-9)
        for index, value in enumerate(update_report.r.data)
    )


def test_mag_update_rejects_non_spd_covariance() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )
    state.m_w_t = [0.4, 0.1, -0.1]
    p: list[float] = _initial_covariance(layout, value=0.1)
    r_raw: list[float] = [
        1.0,
        2.0,
        0.0,
        2.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    mag: MagSample = MagSample(
        frame_id="mag",
        magnetic_field_t=[0.4, 0.1, -0.1],
        magnetic_field_cov=r_raw,
    )
    config: AhrsConfig = _config(mag_gate_d2_threshold=9.0)

    updated_state: AhrsNominalState
    updated_p: list[float]
    update_report: AhrsUpdateData
    updated_state, updated_p, update_report, _ = update_mag(
        layout,
        state,
        p,
        mag,
        config,
        frame_id="mag",
        t_meas=AhrsTime(sec=1, nanosec=0),
    )

    assert update_report.accepted is True
    assert updated_state.q_wb_wxyz == state.q_wb_wxyz
    assert updated_p != p
    assert update_report.r.data == config.mag_r0
