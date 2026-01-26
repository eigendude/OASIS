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
    accel_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.residuals import (
    mag_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.robust_loss import robust_weight
from oasis_control.localization.mounting.state.mounting_state import MountingState
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


def _info_from_cov(cov: NDArray[np.float64]) -> NDArray[np.float64]:
    cov_in: NDArray[np.float64] = np.asarray(cov, dtype=np.float64)
    cov_sym: NDArray[np.float64] = np.asarray(
        0.5 * (cov_in + cov_in.T),
        dtype=np.float64,
    )
    cov_sym = np.asarray(
        cov_sym + np.eye(3, dtype=np.float64) * _COV_EPS,
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


def _accumulate_factor(
    H: NDArray[np.float64],
    b: NDArray[np.float64],
    residual: NDArray[np.float64],
    jacobians: dict[str, NDArray[np.float64]],
    info: NDArray[np.float64],
    mapping: StateMapping,
    weight: float,
) -> tuple[float, float, int]:
    r_w: NDArray[np.float64] = residual * weight
    cost: float = float(0.5 * r_w.T @ info @ r_w)
    res_sq: float = float(r_w @ r_w)
    res_count: int = int(r_w.size)
    for name_i, J_i in jacobians.items():
        block_i: StateBlock = mapping.block(name_i)
        sl_i: slice = block_i.sl()
        b[sl_i] += J_i.T @ info @ r_w
        for name_j, J_j in jacobians.items():
            block_j: StateBlock = mapping.block(name_j)
            sl_j: slice = block_j.sl()
            H[sl_i, sl_j] += J_i.T @ info @ J_j
    return cost, res_sq, res_count


def build_linearization(
    state: MountingState,
    keyframes: tuple[Keyframe, ...],
    params: MountingParams | None,
    *,
    include_translation_vars: bool,
) -> tuple[NDArray[np.float64], NDArray[np.float64], float, dict[str, Any]]:
    """Build the Gauss-Newton linearization for the current state."""
    mapping: StateMapping = StateMapping.from_state(
        state,
        include_translation_vars=include_translation_vars,
    )
    dim: int = mapping.dim()
    H: NDArray[np.float64] = np.zeros((dim, dim), dtype=np.float64)
    b: NDArray[np.float64] = np.zeros(dim, dtype=np.float64)
    cost: float = 0.0
    res_sq_sum: float = 0.0
    res_count: int = 0

    robust_type: str | None = None
    robust_scale_a: float = 1.0
    robust_scale_m: float = 1.0
    if params is not None:
        robust_type = params.solver.robust_loss
        if params.solver.robust_scale_a is not None:
            robust_scale_a = float(params.solver.robust_scale_a)
        if params.solver.robust_scale_m is not None:
            robust_scale_m = float(params.solver.robust_scale_m)

    keyframe_attitudes: dict[int, NDArray[np.float64]] = {
        attitude.keyframe_id: attitude.q_WB_wxyz for attitude in state.keyframes
    }

    for keyframe in keyframes:
        if keyframe.gravity_weight <= 0:
            continue
        a_meas: NDArray[np.float64] = keyframe.gravity_unit_mean_dir_I()
        q_WB_wxyz: NDArray[np.float64] = keyframe_attitudes[keyframe.keyframe_id]
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
            accel_weight,
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
            mag_weight,
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
    return H, b, cost, metrics
