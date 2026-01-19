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
Nominal-state error injection utilities for the AHRS core

The error-state EKF stores small perturbations in a stacked error vector. The
injection step applies a correction ``delta_x`` to the nominal state and
resets the error-state estimate to zero. Attitude and extrinsic rotations are
updated using a small-angle exponential map applied as
``q_new = q_nominal ⊗ Exp(delta_theta)``. The error-state layout controls how
``delta_x`` is partitioned into translation, velocity, attitude, bias, and
sensor transform corrections.
"""

from __future__ import annotations

from oasis_control.localization.ahrs.ahrs_error_state import AhrsErrorStateLayout
from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_quat import quat_from_rotvec_wxyz
from oasis_control.localization.ahrs.ahrs_quat import quat_mul_wxyz
from oasis_control.localization.ahrs.ahrs_quat import quat_normalize_wxyz
from oasis_control.localization.ahrs.ahrs_state import AhrsNominalState
from oasis_control.localization.ahrs.ahrs_types import AhrsSe3Transform


def _require_block_length(name: str, block: list[float], expected: int) -> None:
    if len(block) != expected:
        raise ValueError(f"{name} must have length {expected}")


def _apply_se3_delta(
    transform: AhrsSe3Transform,
    delta_rho: list[float],
    delta_theta: list[float],
) -> AhrsSe3Transform:
    rotation_delta: list[float] = quat_from_rotvec_wxyz(delta_theta)
    rotation_wxyz: list[float] = quat_mul_wxyz(
        list(transform.rotation_wxyz),
        rotation_delta,
    )
    rotation_wxyz = quat_normalize_wxyz(rotation_wxyz)
    return AhrsSe3Transform(
        parent_frame=transform.parent_frame,
        child_frame=transform.child_frame,
        translation_m=[
            transform.translation_m[0] + delta_rho[0],
            transform.translation_m[1] + delta_rho[1],
            transform.translation_m[2] + delta_rho[2],
        ],
        rotation_wxyz=rotation_wxyz,
    )


def inject_error_state(
    layout: AhrsErrorStateLayout,
    state: AhrsNominalState,
    delta_x: list[float],
) -> AhrsNominalState:
    """
    Inject an error-state correction into the nominal state
    """

    if len(delta_x) != layout.dim:
        raise ValueError("delta_x length does not match layout.dim")
    if not is_finite_seq(delta_x):
        raise ValueError("delta_x must contain finite values")

    sl_p: slice = layout.sl_p()
    sl_v: slice = layout.sl_v()
    sl_theta: slice = layout.sl_theta()
    sl_omega: slice = layout.sl_omega()
    sl_bg: slice = layout.sl_bg()
    sl_ba: slice = layout.sl_ba()
    sl_aa: slice = layout.sl_Aa()
    sl_xi_bi: slice = layout.sl_xi_bi()
    sl_xi_bm: slice = layout.sl_xi_bm()
    sl_g: slice = layout.sl_g()
    sl_m: slice = layout.sl_m()

    delta_p: list[float] = delta_x[sl_p]
    delta_v: list[float] = delta_x[sl_v]
    delta_theta: list[float] = delta_x[sl_theta]
    delta_omega: list[float] = delta_x[sl_omega]
    delta_bg: list[float] = delta_x[sl_bg]
    delta_ba: list[float] = delta_x[sl_ba]
    delta_aa: list[float] = delta_x[sl_aa]
    delta_xi_bi: list[float] = delta_x[sl_xi_bi]
    delta_xi_bm: list[float] = delta_x[sl_xi_bm]
    delta_g: list[float] = delta_x[sl_g]
    delta_m: list[float] = delta_x[sl_m]

    _require_block_length("delta_p", delta_p, 3)
    _require_block_length("delta_v", delta_v, 3)
    _require_block_length("delta_theta", delta_theta, 3)
    _require_block_length("delta_omega", delta_omega, 3)
    _require_block_length("delta_bg", delta_bg, 3)
    _require_block_length("delta_ba", delta_ba, 3)
    _require_block_length("delta_aa", delta_aa, 9)
    _require_block_length("delta_xi_bi", delta_xi_bi, 6)
    _require_block_length("delta_xi_bm", delta_xi_bm, 6)
    _require_block_length("delta_g", delta_g, 3)
    _require_block_length("delta_m", delta_m, 3)

    dq_wb: list[float] = quat_from_rotvec_wxyz(delta_theta)
    q_wb: list[float] = quat_mul_wxyz(list(state.q_wb_wxyz), dq_wb)
    q_wb = quat_normalize_wxyz(q_wb)

    t_bi: AhrsSe3Transform = _apply_se3_delta(
        state.t_bi,
        delta_xi_bi[0:3],
        delta_xi_bi[3:6],
    )
    t_bm: AhrsSe3Transform = _apply_se3_delta(
        state.t_bm,
        delta_xi_bm[0:3],
        delta_xi_bm[3:6],
    )

    return AhrsNominalState(
        p_wb_m=[
            state.p_wb_m[0] + delta_p[0],
            state.p_wb_m[1] + delta_p[1],
            state.p_wb_m[2] + delta_p[2],
        ],
        v_wb_mps=[
            state.v_wb_mps[0] + delta_v[0],
            state.v_wb_mps[1] + delta_v[1],
            state.v_wb_mps[2] + delta_v[2],
        ],
        q_wb_wxyz=q_wb,
        omega_wb_rps=[
            state.omega_wb_rps[0] + delta_omega[0],
            state.omega_wb_rps[1] + delta_omega[1],
            state.omega_wb_rps[2] + delta_omega[2],
        ],
        b_g_rps=[
            state.b_g_rps[0] + delta_bg[0],
            state.b_g_rps[1] + delta_bg[1],
            state.b_g_rps[2] + delta_bg[2],
        ],
        b_a_mps2=[
            state.b_a_mps2[0] + delta_ba[0],
            state.b_a_mps2[1] + delta_ba[1],
            state.b_a_mps2[2] + delta_ba[2],
        ],
        a_a=[state.a_a[i] + delta_aa[i] for i in range(9)],
        t_bi=t_bi,
        t_bm=t_bm,
        g_w_mps2=[
            state.g_w_mps2[0] + delta_g[0],
            state.g_w_mps2[1] + delta_g[1],
            state.g_w_mps2[2] + delta_g[2],
        ],
        m_w_t=[
            state.m_w_t[0] + delta_m[0],
            state.m_w_t[1] + delta_m[1],
            state.m_w_t[2] + delta_m[2],
        ],
    )
