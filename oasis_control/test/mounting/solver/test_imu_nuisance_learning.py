################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for learning IMU nuisance parameters from steady segments."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.mounting_types.keyframe import Keyframe
from oasis_control.localization.mounting.solver.optimizer import optimize
from oasis_control.localization.mounting.state.mounting_state import ImuNuisance
from oasis_control.localization.mounting.state.mounting_state import KeyframeAttitude
from oasis_control.localization.mounting.state.mounting_state import MagNuisance
from oasis_control.localization.mounting.state.mounting_state import MountEstimate
from oasis_control.localization.mounting.state.mounting_state import MountingState


# Units: (m/s^2)^2. Meaning: covariance diagonal for raw accel means
_ACCEL_COV_DIAG: float = 1e-3

# Units: (rad/s)^2. Meaning: covariance diagonal for raw gyro means
_OMEGA_COV_DIAG: float = 1e-4


def _raw_keyframe(
    keyframe_id: int,
    q_WB_wxyz: NDArray[np.float64],
    g_W_unit: NDArray[np.float64],
    b_a_true: NDArray[np.float64],
    b_g_true: NDArray[np.float64],
    *,
    g0_mps2: float,
    accel_weight: int,
    omega_weight: int,
) -> Keyframe:
    R_WB: NDArray[np.float64] = Quaternion(q_WB_wxyz).as_matrix()
    R_BI: NDArray[np.float64] = np.eye(3, dtype=np.float64)
    g_I: NDArray[np.float64] = (R_WB.T @ R_BI) @ g_W_unit
    a_corr: NDArray[np.float64] = -g0_mps2 * g_I
    a_raw: NDArray[np.float64] = b_a_true + a_corr
    omega_raw: NDArray[np.float64] = b_g_true
    accel_cov: NDArray[np.float64] = np.eye(3, dtype=np.float64) * _ACCEL_COV_DIAG
    omega_cov: NDArray[np.float64] = np.eye(3, dtype=np.float64) * _OMEGA_COV_DIAG
    return Keyframe(
        keyframe_id=keyframe_id,
        gravity_mean_dir_I=np.zeros(3, dtype=np.float64),
        gravity_cov_dir_I=np.zeros((3, 3), dtype=np.float64),
        gravity_weight=0,
        omega_mean_rads_raw=omega_raw,
        cov_omega_raw=omega_cov,
        omega_weight=omega_weight,
        accel_mean_mps2_raw=a_raw,
        cov_accel_raw=accel_cov,
        accel_weight=accel_weight,
        mag_mean_dir_M=None,
        mag_cov_dir_M=None,
        mag_weight=0,
        segment_count=1,
    )


def test_steady_factors_update_nuisance_estimates() -> None:
    """Ensure steady factors move nuisance estimates toward truth."""
    params: MountingParams = MountingParams.defaults()
    solver_params = replace(params.solver, max_iters=5)
    params = replace(params, solver=solver_params)
    g0_mps2: float = float(params.imu.gravity_mps2)

    g_W_unit: NDArray[np.float64] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    b_a_true: NDArray[np.float64] = np.array([0.15, -0.1, 0.05], dtype=np.float64)
    b_g_true: NDArray[np.float64] = np.array([0.01, -0.02, 0.03], dtype=np.float64)

    q_WB_0: NDArray[np.float64] = np.array(
        [1.0, 0.0, 0.0, 0.0],
        dtype=np.float64,
    )
    q_WB_1: NDArray[np.float64] = Quaternion.from_rotvec(
        np.array([np.pi / 2.0, 0.0, 0.0], dtype=np.float64)
    ).to_wxyz()

    keyframes: tuple[Keyframe, ...] = (
        _raw_keyframe(
            0,
            q_WB_0,
            g_W_unit,
            b_a_true,
            b_g_true,
            g0_mps2=g0_mps2,
            accel_weight=3,
            omega_weight=3,
        ),
        _raw_keyframe(
            1,
            q_WB_1,
            g_W_unit,
            b_a_true,
            b_g_true,
            g0_mps2=g0_mps2,
            accel_weight=3,
            omega_weight=3,
        ),
    )

    mount: MountEstimate = MountEstimate(
        q_BI_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        q_BM_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
    imu: ImuNuisance = ImuNuisance(
        b_a_mps2=np.zeros(3, dtype=np.float64),
        A_a=np.eye(3, dtype=np.float64) * 1.1,
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
        imu_prior=None,
        mag=mag,
        keyframes=(
            KeyframeAttitude(keyframe_id=0, q_WB_wxyz=q_WB_0),
            KeyframeAttitude(keyframe_id=1, q_WB_wxyz=q_WB_1),
        ),
        anchored=False,
        mag_reference_invalid=False,
    )

    updated, _ = optimize(state, keyframes, params)

    b_g_error_initial: float = float(np.linalg.norm(b_g_true - imu.b_g_rads))
    b_g_error_final: float = float(np.linalg.norm(b_g_true - updated.imu.b_g_rads))
    assert b_g_error_final < b_g_error_initial

    b_a_error_initial: float = float(np.linalg.norm(b_a_true - imu.b_a_mps2))
    b_a_error_final: float = float(np.linalg.norm(b_a_true - updated.imu.b_a_mps2))
    assert b_a_error_final < b_a_error_initial

    a_scale_error_initial: float = abs(float(imu.A_a[2, 2] - 1.0))
    a_scale_error_final: float = abs(float(updated.imu.A_a[2, 2] - 1.0))
    assert a_scale_error_final < a_scale_error_initial


def test_posterior_covariance_shrinks_with_weight() -> None:
    """Ensure posterior covariance shrinks as steady weight increases."""
    params: MountingParams = MountingParams.defaults()
    solver_params = replace(params.solver, max_iters=1)
    params = replace(params, solver=solver_params)
    g0_mps2: float = float(params.imu.gravity_mps2)

    g_W_unit: NDArray[np.float64] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    b_a_true: NDArray[np.float64] = np.array([0.05, 0.0, -0.02], dtype=np.float64)
    b_g_true: NDArray[np.float64] = np.array([0.005, -0.01, 0.02], dtype=np.float64)

    q_WB: NDArray[np.float64] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def _run_with_weight(weight: int) -> dict[str, NDArray[np.float64]]:
        keyframes: tuple[Keyframe, ...] = (
            _raw_keyframe(
                0,
                q_WB,
                g_W_unit,
                b_a_true,
                b_g_true,
                g0_mps2=g0_mps2,
                accel_weight=weight,
                omega_weight=weight,
            ),
        )
        mount: MountEstimate = MountEstimate(
            q_BI_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            q_BM_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
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
            imu_prior=None,
            mag=mag,
            keyframes=(KeyframeAttitude(keyframe_id=0, q_WB_wxyz=q_WB),),
            anchored=False,
            mag_reference_invalid=False,
        )
        _, metrics = optimize(state, keyframes, params)
        return metrics

    metrics_low: dict[str, NDArray[np.float64]] = _run_with_weight(1)
    metrics_high: dict[str, NDArray[np.float64]] = _run_with_weight(5)

    cov_low: NDArray[np.float64] = metrics_low["gyro_bias_cov"]
    cov_high: NDArray[np.float64] = metrics_high["gyro_bias_cov"]
    assert float(np.trace(cov_high)) < float(np.trace(cov_low))

    accel_cov: NDArray[np.float64] = metrics_high["accel_param_cov"]
    assert float(np.trace(accel_cov)) > 0.0
