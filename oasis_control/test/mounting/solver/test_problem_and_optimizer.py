################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for problem linearization and optimizer integration."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import SolverParams
from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.mounting_types.keyframe import Keyframe
from oasis_control.localization.mounting.solver.optimizer import optimize
from oasis_control.localization.mounting.solver.problem import build_linearization
from oasis_control.localization.mounting.state.mounting_state import ImuNuisance
from oasis_control.localization.mounting.state.mounting_state import KeyframeAttitude
from oasis_control.localization.mounting.state.mounting_state import MagNuisance
from oasis_control.localization.mounting.state.mounting_state import MountEstimate
from oasis_control.localization.mounting.state.mounting_state import MountingState


# Units: unitless^2. Meaning: covariance diagonal for synthetic gravity data
_GRAVITY_COV_DIAG: float = 1e-4


def _make_keyframe(
    keyframe_id: int,
    q_WB_wxyz: NDArray[np.float64],
    g_W_unit: NDArray[np.float64],
    q_BI_wxyz: NDArray[np.float64],
) -> Keyframe:
    R_WB: NDArray[np.float64] = Quaternion(q_WB_wxyz).as_matrix()
    R_BI: NDArray[np.float64] = Quaternion(q_BI_wxyz).as_matrix()
    a_hat_I: NDArray[np.float64] = (R_WB.T @ R_BI) @ g_W_unit
    gravity_cov: NDArray[np.float64] = np.eye(3, dtype=np.float64) * _GRAVITY_COV_DIAG
    return Keyframe(
        keyframe_id=keyframe_id,
        gravity_mean_dir_I=a_hat_I,
        gravity_cov_dir_I=gravity_cov,
        gravity_weight=1,
        mag_mean_dir_M=None,
        mag_cov_dir_M=None,
        mag_weight=0,
        segment_count=1,
    )


def test_optimizer_reduces_cost_and_handles_mag_optional() -> None:
    """Check a Gauss-Newton step lowers cost without mag variables."""
    g_W_unit: NDArray[np.float64] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    q_BI_true: NDArray[np.float64] = np.array(
        [1.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
    q_BI_init: NDArray[np.float64] = Quaternion.from_rotvec(
        np.array([0.05, 0.0, 0.0], dtype=np.float64)
    ).to_wxyz()
    q_WB_0: NDArray[np.float64] = np.array(
        [1.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
    tilt_rad: float = float(np.deg2rad(30.0))
    q_WB_1: NDArray[np.float64] = Quaternion.from_rotvec(
        np.array([tilt_rad, 0.0, 0.0], dtype=np.float64)
    ).to_wxyz()

    keyframes: tuple[Keyframe, ...] = (
        _make_keyframe(0, q_WB_0, g_W_unit, q_BI_true),
        _make_keyframe(1, q_WB_1, g_W_unit, q_BI_true),
    )
    mount: MountEstimate = MountEstimate(
        q_BI_wxyz=q_BI_init,
        q_BM_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        p_BI_m=np.zeros(3, dtype=np.float64),
        p_BM_m=np.zeros(3, dtype=np.float64),
    )
    imu: ImuNuisance = ImuNuisance(
        b_a_mps2=np.zeros(3, dtype=np.float64),
        A_a=np.eye(3, dtype=np.float64),
        b_g_rads=np.zeros(3, dtype=np.float64),
    )
    mag: MagNuisance = MagNuisance(
        b_m_T=None,
        R_m_unitless2=np.eye(3, dtype=np.float64),
    )
    state: MountingState = MountingState(
        mount=mount,
        g_W_unit=g_W_unit,
        m_W_unit=None,
        imu=imu,
        mag=mag,
        keyframes=(
            KeyframeAttitude(keyframe_id=0, q_WB_wxyz=q_WB_0),
            KeyframeAttitude(keyframe_id=1, q_WB_wxyz=q_WB_1),
        ),
        anchored=False,
        mag_reference_invalid=False,
    )

    params: MountingParams = MountingParams.defaults()
    solver: SolverParams = replace(params.solver, max_iters=1)
    params = replace(params, solver=solver)

    cost_initial: float
    _, _, cost_initial, _ = build_linearization(
        state,
        keyframes,
        params,
        include_translation_vars=False,
    )
    updated_state: MountingState
    updated_state, _ = optimize(
        state,
        keyframes,
        params,
        include_translation_vars=False,
    )
    cost_updated: float
    _, _, cost_updated, _ = build_linearization(
        updated_state,
        keyframes,
        params,
        include_translation_vars=False,
    )

    assert cost_initial > 0.0
    assert cost_updated < cost_initial
