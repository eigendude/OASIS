################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Nominal and covariance propagation for the AHRS core

The prediction layer advances the nominal state and covariance forward in
time between measurement updates. The nominal state follows a simple kinematic
model with zero-mean random-walk terms. The covariance uses a first-order
discretization of the linearized error-state dynamics.

Quaternion conventions:
    * Quaternions are stored in wxyz order
    * q_wb represents the rotation from world to body
    * Composition uses q_new = q_wb ⊗ dq, where dq comes from Exp(omega * dt)
"""

from __future__ import annotations

import math

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_linalg import mat_add
from oasis_control.localization.ahrs.ahrs_linalg import mat_mul
from oasis_control.localization.ahrs.ahrs_linalg import mat_transpose
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState


def _quat_mul(q_left: list[float], q_right: list[float]) -> list[float]:
    """
    Multiply two quaternions in wxyz order
    """

    w1: float = q_left[0]
    x1: float = q_left[1]
    y1: float = q_left[2]
    z1: float = q_left[3]

    w2: float = q_right[0]
    x2: float = q_right[1]
    y2: float = q_right[2]
    z2: float = q_right[3]

    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def _quat_normalize(q_wxyz: list[float]) -> list[float]:
    """
    Normalize a quaternion in wxyz order
    """

    norm: float = math.sqrt(
        q_wxyz[0] * q_wxyz[0]
        + q_wxyz[1] * q_wxyz[1]
        + q_wxyz[2] * q_wxyz[2]
        + q_wxyz[3] * q_wxyz[3]
    )
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be positive")
    inv: float = 1.0 / norm
    return [
        q_wxyz[0] * inv,
        q_wxyz[1] * inv,
        q_wxyz[2] * inv,
        q_wxyz[3] * inv,
    ]


def _quat_from_rotvec(rotvec: list[float]) -> list[float]:
    """
    Build a quaternion from a rotation vector using Exp(omega * dt)

    The rotation vector magnitude is the rotation angle in radians. For small
    angles, the series expansion keeps the result numerically stable.
    """

    rx: float = rotvec[0]
    ry: float = rotvec[1]
    rz: float = rotvec[2]
    angle: float = math.sqrt(rx * rx + ry * ry + rz * rz)

    # Rotation magnitude threshold in rad for small-angle series
    small_angle_rad: float = 1.0e-12

    if angle < small_angle_rad:
        # Small-angle approximation for sin(theta/2) ~= theta/2
        return [1.0, 0.5 * rx, 0.5 * ry, 0.5 * rz]

    # Half-angle term used in quaternion exponential
    half_angle: float = 0.5 * angle
    sin_half: float = math.sin(half_angle)
    inv_angle: float = 1.0 / angle
    return [
        math.cos(half_angle),
        rx * inv_angle * sin_half,
        ry * inv_angle * sin_half,
        rz * inv_angle * sin_half,
    ]


def predict_nominal(state: AhrsNominalState, dt_sec: float) -> AhrsNominalState:
    """
    Propagate the nominal AHRS state forward in time

    Position and velocity follow a constant-acceleration model using the
    gravity vector from the nominal state. Orientation integrates the nominal
    angular rate with a small-angle exponential map.
    """

    if dt_sec < 0.0:
        raise ValueError("dt_sec must be non-negative")
    if dt_sec == 0.0:
        return AhrsNominalState(
            p_wb_m=list(state.p_wb_m),
            v_wb_mps=list(state.v_wb_mps),
            q_wb_wxyz=list(state.q_wb_wxyz),
            omega_wb_rps=list(state.omega_wb_rps),
            b_g_rps=list(state.b_g_rps),
            b_a_mps2=list(state.b_a_mps2),
            a_a=list(state.a_a),
            t_bi=state.t_bi,
            t_bm=state.t_bm,
            g_w_mps2=list(state.g_w_mps2),
            m_w_t=list(state.m_w_t),
        )

    p_wb_m: list[float] = [
        state.p_wb_m[0] + state.v_wb_mps[0] * dt_sec,
        state.p_wb_m[1] + state.v_wb_mps[1] * dt_sec,
        state.p_wb_m[2] + state.v_wb_mps[2] * dt_sec,
    ]
    v_wb_mps: list[float] = [
        state.v_wb_mps[0] + state.g_w_mps2[0] * dt_sec,
        state.v_wb_mps[1] + state.g_w_mps2[1] * dt_sec,
        state.v_wb_mps[2] + state.g_w_mps2[2] * dt_sec,
    ]

    rotvec: list[float] = [
        state.omega_wb_rps[0] * dt_sec,
        state.omega_wb_rps[1] * dt_sec,
        state.omega_wb_rps[2] * dt_sec,
    ]
    dq_wb: list[float] = _quat_from_rotvec(rotvec)
    q_wb_wxyz: list[float] = _quat_normalize(
        _quat_mul(state.q_wb_wxyz, dq_wb)
    )

    return AhrsNominalState(
        p_wb_m=p_wb_m,
        v_wb_mps=v_wb_mps,
        q_wb_wxyz=q_wb_wxyz,
        omega_wb_rps=list(state.omega_wb_rps),
        b_g_rps=list(state.b_g_rps),
        b_a_mps2=list(state.b_a_mps2),
        a_a=list(state.a_a),
        t_bi=state.t_bi,
        t_bm=state.t_bm,
        g_w_mps2=list(state.g_w_mps2),
        m_w_t=list(state.m_w_t),
    )


def _add_isotropic_noise(
    q: list[float], dim: int, sl: slice, intensity: float, dt_sec: float
) -> None:
    """
    Add isotropic random-walk noise to the diagonal of a block
    """

    if intensity <= 0.0 or dt_sec <= 0.0:
        return

    block_dim: int = sl.stop - sl.start
    for i in range(block_dim):
        idx: int = (sl.start + i) * dim + (sl.start + i)
        q[idx] += intensity * dt_sec


def predict_covariance(
    layout: AhrsErrorStateLayout,
    p: list[float],
    dt_sec: float,
    config: AhrsConfig,
) -> list[float]:
    """
    Propagate covariance using a first-order error-state model

    The discrete-time approximation uses
        F = I + A * dt
        P_next = F P F^T + Q
    where Q is built from isotropic random-walk process noise blocks.
    """

    if dt_sec < 0.0:
        raise ValueError("dt_sec must be non-negative")
    if dt_sec == 0.0:
        return list(p)

    dim: int = layout.dim
    if len(p) != dim * dim:
        raise ValueError("p must have length dim * dim")

    f: list[float] = [0.0] * (dim * dim)
    for r in range(dim):
        f[r * dim + r] = 1.0

    sl_p: slice = layout.sl_p()
    sl_v: slice = layout.sl_v()
    sl_theta: slice = layout.sl_theta()
    sl_omega: slice = layout.sl_omega()
    sl_g: slice = layout.sl_g()

    for i in range(3):
        f[(sl_p.start + i) * dim + (sl_v.start + i)] += dt_sec
        f[(sl_v.start + i) * dim + (sl_g.start + i)] += dt_sec
        f[(sl_theta.start + i) * dim + (sl_omega.start + i)] += dt_sec

    temp: list[float] = mat_mul(f, dim, dim, p, dim, dim)
    f_t: list[float] = mat_transpose(f, dim, dim)
    p_next: list[float] = mat_mul(temp, dim, dim, f_t, dim, dim)

    q: list[float] = [0.0] * (dim * dim)
    _add_isotropic_noise(q, dim, layout.sl_v(), config.q_v, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_omega(), config.q_w, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_bg(), config.q_bg, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_ba(), config.q_ba, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_Aa(), config.q_a, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_xi_bi(), config.q_bi, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_xi_bm(), config.q_bm, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_g(), config.q_g, dt_sec)
    _add_isotropic_noise(q, dim, layout.sl_m(), config.q_m, dt_sec)

    p_next = mat_add(p_next, q)
    return symmetrize(p_next, dim)


def predict_step(
    layout: AhrsErrorStateLayout,
    state: AhrsNominalState,
    p: list[float],
    dt_sec: float,
    config: AhrsConfig,
) -> tuple[AhrsNominalState, list[float]]:
    """
    Propagate nominal state and covariance over a time step
    """

    x_next: AhrsNominalState = predict_nominal(state, dt_sec)
    p_next: list[float] = predict_covariance(layout, p, dt_sec, config)
    return x_next, p_next
