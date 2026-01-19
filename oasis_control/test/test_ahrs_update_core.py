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

from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_inject import inject_error_state
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_state import default_nominal_state
from oasis_control.localization.ahrs.ahrs_types import AhrsTime
from oasis_control.localization.ahrs.ahrs_types import AhrsUpdateData
from oasis_control.localization.ahrs.ahrs_update_core import kalman_update


def _diag_covariance(layout: AhrsErrorStateLayout) -> list[float]:
    dim: int = layout.dim
    p: list[float] = [0.0] * (dim * dim)
    for i in range(dim):
        p[i * dim + i] = 0.0
    return p


def _set_diag(p: list[float], dim: int, index: int, value: float) -> None:
    p[index * dim + index] = value


def test_ahrs_update_core_gyro_equivalence() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    dim: int = layout.dim
    p: list[float] = _diag_covariance(layout)

    omega_var: float = 0.2
    bg_var: float = 0.05
    sl_omega: slice = layout.sl_omega()
    sl_bg: slice = layout.sl_bg()
    for i in range(3):
        _set_diag(p, dim, sl_omega.start + i, omega_var)
        _set_diag(p, dim, sl_bg.start + i, bg_var)

    z: list[float] = [1.0, 0.0, 0.0]
    z_hat: list[float] = [0.0, 0.0, 0.0]
    nu: list[float] = [1.0, 0.0, 0.0]

    h: list[float] = [0.0] * (3 * dim)
    for i in range(3):
        h[i * dim + sl_omega.start + i] = 1.0
        h[i * dim + sl_bg.start + i] = 1.0

    r: list[float] = [
        0.1,
        0.0,
        0.0,
        0.0,
        0.1,
        0.0,
        0.0,
        0.0,
        0.1,
    ]

    updated_p: list[float]
    report: AhrsUpdateData
    accepted: bool
    delta_x: list[float]
    updated_p, report, accepted, delta_x = kalman_update(
        layout,
        p,
        z=z,
        z_hat=z_hat,
        nu=nu,
        h=h,
        r=r,
        gate_threshold=10.0,
        sensor="gyro",
        frame_id="imu",
        t_meas=AhrsTime(sec=0, nanosec=0),
    )

    expected_s: float = omega_var + bg_var + r[0]
    expected_gain_omega: float = omega_var / expected_s
    expected_gain_bg: float = bg_var / expected_s

    assert report is not None
    assert accepted is True
    assert math.isclose(
        delta_x[sl_omega.start],
        expected_gain_omega,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        delta_x[sl_bg.start],
        expected_gain_bg,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    assert updated_p[sl_omega.start * dim + sl_omega.start] < omega_var
    assert updated_p[sl_bg.start * dim + sl_bg.start] < bg_var


def test_inject_error_state_theta_updates_quaternion() -> None:
    layout: AhrsErrorStateLayout = AhrsErrorStateLayout()
    state: AhrsNominalState = default_nominal_state(
        body_frame_id="base_link",
        imu_frame_id="imu",
        mag_frame_id="mag",
    )

    delta_x: list[float] = [0.0] * layout.dim
    sl_theta: slice = layout.sl_theta()
    delta_x[sl_theta.start] = 1.0e-3
    delta_x[sl_theta.start + 1] = -2.0e-3
    delta_x[sl_theta.start + 2] = 0.5e-3

    updated_state: AhrsNominalState = inject_error_state(layout, state, delta_x)

    rx: float = delta_x[sl_theta.start]
    ry: float = delta_x[sl_theta.start + 1]
    rz: float = delta_x[sl_theta.start + 2]
    angle: float = math.sqrt(rx * rx + ry * ry + rz * rz)

    half_angle: float = 0.5 * angle
    if angle == 0.0:
        expected_q: list[float] = [1.0, 0.0, 0.0, 0.0]
    else:
        sin_half: float = math.sin(half_angle)
        inv_angle: float = 1.0 / angle
        expected_q = [
            math.cos(half_angle),
            rx * inv_angle * sin_half,
            ry * inv_angle * sin_half,
            rz * inv_angle * sin_half,
        ]

    norm: float = math.sqrt(
        updated_state.q_wb_wxyz[0] * updated_state.q_wb_wxyz[0]
        + updated_state.q_wb_wxyz[1] * updated_state.q_wb_wxyz[1]
        + updated_state.q_wb_wxyz[2] * updated_state.q_wb_wxyz[2]
        + updated_state.q_wb_wxyz[3] * updated_state.q_wb_wxyz[3]
    )

    assert math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(
        updated_state.q_wb_wxyz[0],
        expected_q[0],
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        updated_state.q_wb_wxyz[1],
        expected_q[1],
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        updated_state.q_wb_wxyz[2],
        expected_q[2],
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        updated_state.q_wb_wxyz[3],
        expected_q[3],
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
