################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Gauss-Newton optimizer for mounting calibration."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.math_utils.linalg import SO3
from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.mounting_types.keyframe import Keyframe
from oasis_control.localization.mounting.solver.problem import build_linearization
from oasis_control.localization.mounting.state.mounting_state import ImuNuisance
from oasis_control.localization.mounting.state.mounting_state import KeyframeAttitude
from oasis_control.localization.mounting.state.mounting_state import MagNuisance
from oasis_control.localization.mounting.state.mounting_state import MountEstimate
from oasis_control.localization.mounting.state.mounting_state import MountingState
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_A_A
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_A
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_G
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_M
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_G_W
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_M_W
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_R_BI
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_R_BM
from oasis_control.localization.mounting.state.state_mapping import StateBlock
from oasis_control.localization.mounting.state.state_mapping import StateMapping


# Units: unitless. Meaning: Levenberg-Marquardt diagonal damping
_LM_DAMPING: float = 1e-6


def _apply_rotation(
    q_wxyz: NDArray[np.float64],
    delta: NDArray[np.float64],
) -> NDArray[np.float64]:
    dq: Quaternion = Quaternion.from_rotvec(delta)
    q: Quaternion = Quaternion(q_wxyz).normalized()
    return (dq * q).normalized().to_wxyz()


def _apply_direction(
    direction: NDArray[np.float64],
    delta: NDArray[np.float64],
) -> NDArray[np.float64]:
    updated: NDArray[np.float64] = SO3.exp(delta) @ direction
    norm: float = float(np.linalg.norm(updated))
    if norm <= 0.0:
        return direction
    return updated / norm


def _solve(H: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    rhs: NDArray[np.float64] = -b
    try:
        delta: NDArray[np.float64] = np.asarray(
            np.linalg.solve(H, rhs),
            dtype=np.float64,
        )
    except np.linalg.LinAlgError:
        dim: int = int(H.shape[0])
        H_damped: NDArray[np.float64] = H + np.eye(dim, dtype=np.float64) * _LM_DAMPING
        try:
            delta = np.asarray(
                np.linalg.solve(H_damped, rhs),
                dtype=np.float64,
            )
        except np.linalg.LinAlgError:
            delta = np.asarray(
                np.linalg.lstsq(H_damped, rhs, rcond=None)[0],
                dtype=np.float64,
            )
    return delta


def _apply_delta(
    state: MountingState,
    delta: NDArray[np.float64],
    mapping: StateMapping,
) -> MountingState:
    q_BI_wxyz: NDArray[np.float64] = state.mount.q_BI_wxyz
    if mapping.has(BLOCK_NAME_R_BI):
        block_r_bi: StateBlock = mapping.block(BLOCK_NAME_R_BI)
        q_BI_wxyz = _apply_rotation(q_BI_wxyz, delta[block_r_bi.sl()])

    q_BM_wxyz: NDArray[np.float64] = state.mount.q_BM_wxyz
    if mapping.has(BLOCK_NAME_R_BM):
        block_r_bm: StateBlock = mapping.block(BLOCK_NAME_R_BM)
        q_BM_wxyz = _apply_rotation(q_BM_wxyz, delta[block_r_bm.sl()])

    g_W_unit: NDArray[np.float64] = state.g_W_unit
    if mapping.has(BLOCK_NAME_G_W):
        block_g_w: StateBlock = mapping.block(BLOCK_NAME_G_W)
        g_W_unit = _apply_direction(g_W_unit, delta[block_g_w.sl()])

    m_W_unit: NDArray[np.float64] | None = state.m_W_unit
    if m_W_unit is not None and mapping.has(BLOCK_NAME_M_W):
        block_m_w: StateBlock = mapping.block(BLOCK_NAME_M_W)
        m_W_unit = _apply_direction(m_W_unit, delta[block_m_w.sl()])

    b_a_mps2: NDArray[np.float64] = state.imu.b_a_mps2
    if mapping.has(BLOCK_NAME_B_A):
        block_b_a: StateBlock = mapping.block(BLOCK_NAME_B_A)
        b_a_mps2 = b_a_mps2 + delta[block_b_a.sl()]

    A_a: NDArray[np.float64] = state.imu.A_a
    if mapping.has(BLOCK_NAME_A_A):
        block_a_a: StateBlock = mapping.block(BLOCK_NAME_A_A)
        delta_A: NDArray[np.float64] = delta[block_a_a.sl()].reshape(3, 3)
        A_a = A_a + delta_A

    b_g_rads: NDArray[np.float64] = state.imu.b_g_rads
    if mapping.has(BLOCK_NAME_B_G):
        block_b_g: StateBlock = mapping.block(BLOCK_NAME_B_G)
        b_g_rads = b_g_rads + delta[block_b_g.sl()]

    b_m_T: NDArray[np.float64] | None = state.mag.b_m_T
    if b_m_T is not None and mapping.has(BLOCK_NAME_B_M):
        block_b_m: StateBlock = mapping.block(BLOCK_NAME_B_M)
        b_m_T = b_m_T + delta[block_b_m.sl()]

    updated_keyframes: list[KeyframeAttitude] = []
    for attitude in state.keyframes:
        block_wb: StateBlock = mapping.keyframe_block(attitude.keyframe_id)
        q_WB_updated: NDArray[np.float64] = _apply_rotation(
            attitude.q_WB_wxyz,
            delta[block_wb.sl()],
        )
        updated_keyframes.append(
            KeyframeAttitude(
                keyframe_id=attitude.keyframe_id,
                q_WB_wxyz=q_WB_updated,
            )
        )

    mount: MountEstimate = MountEstimate(
        q_BI_wxyz=q_BI_wxyz,
        q_BM_wxyz=q_BM_wxyz,
    )
    imu: ImuNuisance = ImuNuisance(
        b_a_mps2=b_a_mps2,
        A_a=A_a,
        b_g_rads=b_g_rads,
    )
    mag: MagNuisance = MagNuisance(
        b_m_T=b_m_T,
        R_m_unitless2=state.mag.R_m_unitless2,
    )
    return replace(
        state,
        mount=mount,
        g_W_unit=g_W_unit,
        m_W_unit=m_W_unit,
        imu=imu,
        mag=mag,
        keyframes=tuple(updated_keyframes),
    )


def optimize(
    state: MountingState,
    keyframes: tuple[Keyframe, ...],
    params: MountingParams | None,
) -> tuple[MountingState, dict[str, Any]]:
    """Run Gauss-Newton iterations and return the updated state."""
    max_iters: int = 1
    if params is not None and params.solver.max_iters is not None:
        max_iters = int(params.solver.max_iters)
    max_iters = max(max_iters, 1)

    current: MountingState = state
    initial_cost: float = 0.0
    final_cost: float = 0.0
    metrics: dict[str, Any] = {}

    for iteration in range(max_iters):
        mapping: StateMapping = StateMapping.from_state(current)
        H, b, cost, metrics = build_linearization(
            current,
            keyframes,
            params,
        )
        if iteration == 0:
            initial_cost = cost
        delta: NDArray[np.float64] = _solve(H, b)
        current = _apply_delta(current, delta, mapping)

    _, _, final_cost, metrics = build_linearization(
        current,
        keyframes,
        params,
    )

    report: dict[str, Any] = {
        "initial_cost": initial_cost,
        "final_cost": final_cost,
        "iterations": max_iters,
    }
    report.update(metrics)
    return current, report
