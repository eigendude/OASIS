################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Linearization builder for mounting calibration solver."""

from __future__ import annotations

from typing import Any

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.mounting_types.keyframe import Keyframe
from oasis_control.localization.mounting.solver.residuals import (
    accel_gravity_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.residuals import (
    accel_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.residuals import (
    mag_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.residuals import (
    steady_gyro_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.robust_loss import robust_weight
from oasis_control.localization.mounting.state.mounting_state import MountingState
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_A_A
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_A
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_G
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_B_M
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_G_W
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_M_W
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_R_BI
from oasis_control.localization.mounting.state.state_mapping import BLOCK_NAME_R_BM
from oasis_control.localization.mounting.state.state_mapping import (
    BLOCK_NAME_R_WB_PREFIX,
)
from oasis_control.localization.mounting.state.state_mapping import StateBlock
from oasis_control.localization.mounting.state.state_mapping import StateMapping


# Units: unitless. Meaning: diagonal jitter for covariance inversion
_COV_EPS: float = 1e-9

# Units: unitless. Meaning: weak diagonal prior for nuisance blocks
_NUISANCE_PRIOR_LAMBDA: float = 1e-6

# Units: unitless. Meaning: weak diagonal prior for direction blocks
_DIRECTION_PRIOR_LAMBDA: float = 1e-6


def _info_from_cov(cov: NDArray[np.float64]) -> NDArray[np.float64]:
    cov_in: NDArray[np.float64] = np.asarray(cov, dtype=np.float64)
    dim: int = int(cov_in.shape[0])
    cov_sym: NDArray[np.float64] = np.asarray(
        0.5 * (cov_in + cov_in.T),
        dtype=np.float64,
    )
    cov_sym = np.asarray(
        cov_sym + np.eye(dim, dtype=np.float64) * _COV_EPS,
        dtype=np.float64,
    )
    try:
        info: NDArray[np.float64] = np.asarray(
            np.linalg.inv(cov_sym),
            dtype=np.float64,
        )
    except np.linalg.LinAlgError:
        info = np.asarray(
            np.linalg.pinv(cov_sym),
            dtype=np.float64,
        )
    return info


def _info_from_variance(variance: float) -> NDArray[np.float64]:
    var: float = float(variance)
    if not np.isfinite(var) or var <= 0.0:
        var = _COV_EPS
    return np.array([[1.0 / var]], dtype=np.float64)


def _accumulate_factor(
    H: NDArray[np.float64],
    b: NDArray[np.float64],
    residual: NDArray[np.float64],
    jacobians: dict[str, NDArray[np.float64]],
    info: NDArray[np.float64],
    mapping: StateMapping,
    weight: float,
) -> tuple[float, float, int]:
    sqrt_weight: float = float(np.sqrt(weight))
    r_w: NDArray[np.float64] = residual * sqrt_weight
    cost: float = float(0.5 * r_w.T @ info @ r_w)
    res_sq: float = float(r_w @ r_w)
    res_count: int = int(r_w.size)
    for name_i, J_i in jacobians.items():
        block_i: StateBlock = mapping.block(name_i)
        sl_i: slice = block_i.sl()
        Jw_i: NDArray[np.float64] = J_i * sqrt_weight
        b[sl_i] += Jw_i.T @ info @ r_w
        for name_j, J_j in jacobians.items():
            block_j: StateBlock = mapping.block(name_j)
            sl_j: slice = block_j.sl()
            Jw_j: NDArray[np.float64] = J_j * sqrt_weight
            H[sl_i, sl_j] += Jw_i.T @ info @ Jw_j
    return cost, res_sq, res_count


def _add_prior_block(
    H: NDArray[np.float64],
    b: NDArray[np.float64],
    mapping: StateMapping,
    names: tuple[str, ...],
    residual: NDArray[np.float64],
    info: NDArray[np.float64],
) -> None:
    indices: list[int] = []
    for name in names:
        if not mapping.has(name):
            return
        block: StateBlock = mapping.block(name)
        indices.extend(range(block.start, block.stop()))
    if not indices:
        return
    idx: NDArray[np.int64] = np.asarray(indices, dtype=np.int64)
    H[np.ix_(idx, idx)] += info
    b[idx] += info @ residual


def _apply_weak_prior(
    H: NDArray[np.float64],
    mapping: StateMapping,
    name: str,
    weight: float,
) -> None:
    if not mapping.has(name):
        return
    block: StateBlock = mapping.block(name)
    sl: slice = block.sl()
    H[sl, sl] += np.eye(block.dim, dtype=np.float64) * weight


def build_linearization(
    state: MountingState,
    keyframes: tuple[Keyframe, ...],
    params: MountingParams | None,
) -> tuple[NDArray[np.float64], NDArray[np.float64], float, dict[str, Any]]:
    """Build the Gauss-Newton linearization for the current state."""
    mapping: StateMapping = StateMapping.from_state(state)
    dim: int = mapping.dim()
    H: NDArray[np.float64] = np.zeros((dim, dim), dtype=np.float64)
    b: NDArray[np.float64] = np.zeros(dim, dtype=np.float64)
    cost: float = 0.0
    res_sq_sum: float = 0.0
    res_count: int = 0

    robust_type: str | None = None
    robust_scale_a: float = 1.0
    robust_scale_m: float = 1.0
    g0_mps2: float = 9.80665
    if params is not None:
        robust_type = params.solver.robust_loss
        if params.solver.robust_scale_a is not None:
            robust_scale_a = float(params.solver.robust_scale_a)
        if params.solver.robust_scale_m is not None:
            robust_scale_m = float(params.solver.robust_scale_m)
        g0_mps2 = float(params.imu.gravity_mps2)

    keyframe_attitudes: dict[int, NDArray[np.float64]] = {
        attitude.keyframe_id: attitude.q_WB_wxyz for attitude in state.keyframes
    }

    for keyframe in keyframes:
        q_WB_wxyz: NDArray[np.float64] = keyframe_attitudes[keyframe.keyframe_id]
        if keyframe.gravity_weight > 0:
            a_meas: NDArray[np.float64] = keyframe.gravity_unit_mean_dir_I()
            accel = accel_residual_with_jacobians(
                a_meas,
                q_WB_wxyz,
                state.mount.q_BI_wxyz,
                state.g_W_unit,
            )
            accel_cov: NDArray[np.float64] = keyframe.gravity_cov_dir_I
            accel_info: NDArray[np.float64] = _info_from_cov(accel_cov)
            accel_weight: float = robust_weight(
                accel.residual,
                robust_type,
                robust_scale_a,
            )
            total_accel_weight: float = float(keyframe.gravity_weight) * accel_weight
            if total_accel_weight > 0.0:
                jacobians: dict[str, NDArray[np.float64]] = {
                    BLOCK_NAME_R_BI: accel.J_bi,
                    BLOCK_NAME_G_W: accel.J_gw,
                    f"{BLOCK_NAME_R_WB_PREFIX}_{keyframe.keyframe_id}": accel.J_wb,
                }
                factor_cost, factor_sq, factor_count = _accumulate_factor(
                    H,
                    b,
                    accel.residual,
                    jacobians,
                    accel_info,
                    mapping,
                    total_accel_weight,
                )
                cost += factor_cost
                res_sq_sum += factor_sq
                res_count += factor_count

        if keyframe.omega_weight > 0:
            gyro = steady_gyro_residual_with_jacobians(
                keyframe.omega_mean_rads_raw,
                state.imu.b_g_rads,
            )
            omega_cov: NDArray[np.float64] = keyframe.cov_omega_raw
            omega_info: NDArray[np.float64] = _info_from_cov(omega_cov)
            omega_weight: float = robust_weight(
                gyro.residual,
                robust_type,
                robust_scale_a,
            )
            total_omega_weight: float = float(keyframe.omega_weight) * omega_weight
            if total_omega_weight > 0.0:
                gyro_jacobians: dict[str, NDArray[np.float64]] = {
                    BLOCK_NAME_B_G: gyro.J_bg,
                }
                factor_cost, factor_sq, factor_count = _accumulate_factor(
                    H,
                    b,
                    gyro.residual,
                    gyro_jacobians,
                    omega_info,
                    mapping,
                    total_omega_weight,
                )
                cost += factor_cost
                res_sq_sum += factor_sq
                res_count += factor_count

        if keyframe.accel_weight > 0:
            try:
                accel_raw = accel_gravity_residual_with_jacobians(
                    keyframe.accel_mean_mps2_raw,
                    state.imu.b_a_mps2,
                    state.imu.A_a,
                    q_WB_wxyz,
                    state.mount.q_BI_wxyz,
                    state.g_W_unit,
                    g0_mps2=g0_mps2,
                )
            except ValueError:
                accel_raw = None
            if accel_raw is not None:
                cov_raw: NDArray[np.float64] = keyframe.cov_accel_raw
                cov_corr: NDArray[np.float64] = (
                    state.imu.A_a @ cov_raw @ state.imu.A_a.T
                )
                accel_raw_info: NDArray[np.float64] = _info_from_cov(cov_corr)
                accel_raw_weight: float = robust_weight(
                    accel_raw.residual_dir,
                    robust_type,
                    robust_scale_a,
                )
                total_accel_raw_weight: float = (
                    float(keyframe.accel_weight) * accel_raw_weight
                )
                if total_accel_raw_weight > 0.0:
                    accel_jacobians = {
                        BLOCK_NAME_R_BI: accel_raw.J_bi_dir,
                        BLOCK_NAME_G_W: accel_raw.J_gw_dir,
                        BLOCK_NAME_B_A: accel_raw.J_ba_dir,
                        BLOCK_NAME_A_A: accel_raw.J_Aa_dir,
                        f"{BLOCK_NAME_R_WB_PREFIX}_{keyframe.keyframe_id}": (
                            accel_raw.J_wb_dir
                        ),
                    }
                    factor_cost, factor_sq, factor_count = _accumulate_factor(
                        H,
                        b,
                        accel_raw.residual_dir,
                        accel_jacobians,
                        accel_raw_info,
                        mapping,
                        total_accel_raw_weight,
                    )
                    cost += factor_cost
                    res_sq_sum += factor_sq
                    res_count += factor_count

                a_norm: float = float(np.linalg.norm(accel_raw.a_corr))
                if a_norm > 0.0:
                    a_unit: NDArray[np.float64] = accel_raw.a_corr / a_norm
                    variance: float = float(a_unit.T @ cov_corr @ a_unit)
                    mag_raw_info: NDArray[np.float64] = _info_from_variance(variance)
                    mag_raw_weight: float = robust_weight(
                        accel_raw.residual_mag,
                        robust_type,
                        robust_scale_a,
                    )
                    total_mag_raw_weight = float(keyframe.accel_weight) * mag_raw_weight
                    if total_mag_raw_weight > 0.0:
                        mag_raw_jacobians = {
                            BLOCK_NAME_B_A: accel_raw.J_ba_mag,
                            BLOCK_NAME_A_A: accel_raw.J_Aa_mag,
                        }
                        factor_cost, factor_sq, factor_count = _accumulate_factor(
                            H,
                            b,
                            accel_raw.residual_mag,
                            mag_raw_jacobians,
                            mag_raw_info,
                            mapping,
                            total_mag_raw_weight,
                        )
                        cost += factor_cost
                        res_sq_sum += factor_sq
                        res_count += factor_count

        if (
            keyframe.mag_mean_dir_M is None
            or keyframe.mag_cov_dir_M is None
            or keyframe.mag_weight <= 0
            or state.m_W_unit is None
        ):
            continue
        mag = mag_residual_with_jacobians(
            keyframe.mag_unit_mean_dir_M(),
            q_WB_wxyz,
            state.mount.q_BM_wxyz,
            state.m_W_unit,
        )
        mag_cov: NDArray[np.float64] = keyframe.mag_cov_dir_M
        mag_info: NDArray[np.float64] = _info_from_cov(mag_cov)
        mag_weight: float = robust_weight(
            mag.residual,
            robust_type,
            robust_scale_m,
        )
        total_mag_weight: float = float(keyframe.mag_weight) * mag_weight
        if total_mag_weight <= 0.0:
            continue
        mag_jacobians: dict[str, NDArray[np.float64]] = {
            BLOCK_NAME_R_BM: mag.J_bm,
            BLOCK_NAME_M_W: mag.J_mw,
            f"{BLOCK_NAME_R_WB_PREFIX}_{keyframe.keyframe_id}": mag.J_wb,
        }
        factor_cost, factor_sq, factor_count = _accumulate_factor(
            H,
            b,
            mag.residual,
            mag_jacobians,
            mag_info,
            mapping,
            total_mag_weight,
        )
        cost += factor_cost
        res_sq_sum += factor_sq
        res_count += factor_count

    rms: float
    if res_count <= 0:
        rms = 0.0
    else:
        rms = float(np.sqrt(res_sq_sum / float(res_count)))
    metrics: dict[str, Any] = {
        "rms": rms,
        "residual_count": res_count,
    }
    if state.imu_prior is not None:
        if state.imu_prior.cov_b_g is not None:
            b_g_residual: NDArray[np.float64] = (
                state.imu.b_g_rads - state.imu_prior.b_g_rads
            )
            b_g_info: NDArray[np.float64] = _info_from_cov(state.imu_prior.cov_b_g)
            _add_prior_block(
                H,
                b,
                mapping,
                (BLOCK_NAME_B_G,),
                b_g_residual,
                b_g_info,
            )
        if state.imu_prior.cov_a_params is not None:
            a_prior: NDArray[np.float64] = np.concatenate(
                [
                    state.imu_prior.b_a_mps2,
                    state.imu_prior.A_a.reshape(9),
                ]
            )
            a_state: NDArray[np.float64] = np.concatenate(
                [
                    state.imu.b_a_mps2,
                    state.imu.A_a.reshape(9),
                ]
            )
            a_residual: NDArray[np.float64] = a_state - a_prior
            a_info: NDArray[np.float64] = _info_from_cov(state.imu_prior.cov_a_params)
            _add_prior_block(
                H,
                b,
                mapping,
                (BLOCK_NAME_B_A, BLOCK_NAME_A_A),
                a_residual,
                a_info,
            )

    if state.imu_prior is None or state.imu_prior.cov_a_params is None:
        A_a_target: NDArray[np.float64] = np.eye(3, dtype=np.float64).reshape(9)
        A_a_state: NDArray[np.float64] = state.imu.A_a.reshape(9)
        A_a_residual: NDArray[np.float64] = A_a_state - A_a_target
        A_a_info: NDArray[np.float64] = (
            np.eye(9, dtype=np.float64) * _NUISANCE_PRIOR_LAMBDA
        )
        _add_prior_block(
            H,
            b,
            mapping,
            (BLOCK_NAME_A_A,),
            A_a_residual,
            A_a_info,
        )
    _apply_weak_prior(H, mapping, BLOCK_NAME_G_W, _DIRECTION_PRIOR_LAMBDA)
    _apply_weak_prior(H, mapping, BLOCK_NAME_M_W, _DIRECTION_PRIOR_LAMBDA)
    _apply_weak_prior(H, mapping, BLOCK_NAME_B_A, _NUISANCE_PRIOR_LAMBDA)
    _apply_weak_prior(H, mapping, BLOCK_NAME_B_G, _NUISANCE_PRIOR_LAMBDA)
    _apply_weak_prior(H, mapping, BLOCK_NAME_B_M, _NUISANCE_PRIOR_LAMBDA)
    return H, b, cost, metrics
