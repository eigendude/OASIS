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

from oasis_control.localization.ahrs.ahrs_config import AhrsConfig
from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_linalg import mat_add
from oasis_control.localization.ahrs.ahrs_linalg import mat_mul
from oasis_control.localization.ahrs.ahrs_linalg import mat_transpose
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize
from oasis_control.localization.ahrs.ahrs_quat import quat_from_rotvec_wxyz
from oasis_control.localization.ahrs.ahrs_quat import quat_mul_wxyz
from oasis_control.localization.ahrs.ahrs_quat import quat_normalize_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState


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
    dq_wb: list[float] = quat_from_rotvec_wxyz(rotvec)
    # Normalize after composition to remove any residual numerical drift
    q_wb_wxyz: list[float] = quat_normalize_wxyz(quat_mul_wxyz(state.q_wb_wxyz, dq_wb))

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

    # Minimal model: p <- v, v <- g, theta <- omega, coupling deferred
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
