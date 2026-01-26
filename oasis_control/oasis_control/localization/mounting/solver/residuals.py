################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Residual and Jacobian helpers for mounting calibration solver.

Jacobians use left perturbations with q' = exp(δ) ⊗ q
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.linalg import SO3
from oasis_control.localization.mounting.math_utils.quat import Quaternion


def _unit(vec: NDArray[np.float64]) -> NDArray[np.float64]:
    norm: float = float(np.linalg.norm(vec))
    if norm <= 0.0 or not np.isfinite(norm):
        raise ValueError("vector must be non-zero")
    return vec / norm


def _quat_to_matrix(q_wxyz: NDArray[np.float64]) -> NDArray[np.float64]:
    return Quaternion(q_wxyz).normalized().as_matrix()


@dataclass(frozen=True)
class AccelResidual:
    """Container for accel residual and Jacobians."""

    residual: NDArray[np.float64]
    prediction: NDArray[np.float64]
    J_wb: NDArray[np.float64]
    J_bi: NDArray[np.float64]
    J_gw: NDArray[np.float64]


@dataclass(frozen=True)
class MagResidual:
    """Container for mag residual and Jacobians."""

    residual: NDArray[np.float64]
    prediction: NDArray[np.float64]
    J_wb: NDArray[np.float64]
    J_bm: NDArray[np.float64]
    J_mw: NDArray[np.float64]


def accel_residual(
    a_hat_I_meas: NDArray[np.float64],
    q_WB_wxyz: NDArray[np.float64],
    q_BI_wxyz: NDArray[np.float64],
    g_W_unit: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return accel residual and predicted direction in the IMU frame."""
    a_meas: NDArray[np.float64] = _unit(np.asarray(a_hat_I_meas, dtype=np.float64))
    g_unit: NDArray[np.float64] = _unit(np.asarray(g_W_unit, dtype=np.float64))
    R_WB: NDArray[np.float64] = _quat_to_matrix(q_WB_wxyz)
    R_BI: NDArray[np.float64] = _quat_to_matrix(q_BI_wxyz)
    a_hat_pred_I: NDArray[np.float64] = (R_WB.T @ R_BI) @ g_unit
    residual: NDArray[np.float64] = np.cross(a_meas, a_hat_pred_I)
    return residual.astype(np.float64), a_hat_pred_I.astype(np.float64)


def mag_residual(
    m_hat_M_meas: NDArray[np.float64],
    q_WB_wxyz: NDArray[np.float64],
    q_BM_wxyz: NDArray[np.float64],
    m_W_unit: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return mag residual and predicted direction in the mag frame."""
    m_meas: NDArray[np.float64] = _unit(np.asarray(m_hat_M_meas, dtype=np.float64))
    m_unit: NDArray[np.float64] = _unit(np.asarray(m_W_unit, dtype=np.float64))
    R_WB: NDArray[np.float64] = _quat_to_matrix(q_WB_wxyz)
    R_BM: NDArray[np.float64] = _quat_to_matrix(q_BM_wxyz)
    m_hat_pred_M: NDArray[np.float64] = (R_WB.T @ R_BM) @ m_unit
    residual: NDArray[np.float64] = np.cross(m_meas, m_hat_pred_M)
    return residual.astype(np.float64), m_hat_pred_M.astype(np.float64)


def accel_residual_with_jacobians(
    a_hat_I_meas: NDArray[np.float64],
    q_WB_wxyz: NDArray[np.float64],
    q_BI_wxyz: NDArray[np.float64],
    g_W_unit: NDArray[np.float64],
) -> AccelResidual:
    """Return accel residual and Jacobians for left perturbations."""
    residual, a_hat_pred_I = accel_residual(
        a_hat_I_meas,
        q_WB_wxyz,
        q_BI_wxyz,
        g_W_unit,
    )
    a_meas: NDArray[np.float64] = _unit(np.asarray(a_hat_I_meas, dtype=np.float64))
    R_WB: NDArray[np.float64] = _quat_to_matrix(q_WB_wxyz)
    R_BI: NDArray[np.float64] = _quat_to_matrix(q_BI_wxyz)
    g_unit: NDArray[np.float64] = _unit(np.asarray(g_W_unit, dtype=np.float64))
    pred_hat: NDArray[np.float64] = a_hat_pred_I
    hat_meas: NDArray[np.float64] = a_meas

    J_pred_wb: NDArray[np.float64] = SO3.hat(pred_hat) @ R_WB.T
    J_pred_bi: NDArray[np.float64] = -SO3.hat(pred_hat) @ R_WB.T
    J_pred_gw: NDArray[np.float64] = -R_WB.T @ R_BI @ SO3.hat(g_unit)

    J_wb: NDArray[np.float64] = SO3.hat(hat_meas) @ J_pred_wb
    J_bi: NDArray[np.float64] = SO3.hat(hat_meas) @ J_pred_bi
    J_gw: NDArray[np.float64] = SO3.hat(hat_meas) @ J_pred_gw

    return AccelResidual(
        residual=residual,
        prediction=pred_hat,
        J_wb=J_wb.astype(np.float64),
        J_bi=J_bi.astype(np.float64),
        J_gw=J_gw.astype(np.float64),
    )


def mag_residual_with_jacobians(
    m_hat_M_meas: NDArray[np.float64],
    q_WB_wxyz: NDArray[np.float64],
    q_BM_wxyz: NDArray[np.float64],
    m_W_unit: NDArray[np.float64],
) -> MagResidual:
    """Return mag residual and Jacobians for left perturbations."""
    residual, m_hat_pred_M = mag_residual(
        m_hat_M_meas,
        q_WB_wxyz,
        q_BM_wxyz,
        m_W_unit,
    )
    m_meas: NDArray[np.float64] = _unit(np.asarray(m_hat_M_meas, dtype=np.float64))
    R_WB: NDArray[np.float64] = _quat_to_matrix(q_WB_wxyz)
    R_BM: NDArray[np.float64] = _quat_to_matrix(q_BM_wxyz)
    m_unit: NDArray[np.float64] = _unit(np.asarray(m_W_unit, dtype=np.float64))
    pred_hat: NDArray[np.float64] = m_hat_pred_M
    hat_meas: NDArray[np.float64] = m_meas

    J_pred_wb: NDArray[np.float64] = SO3.hat(pred_hat) @ R_WB.T
    J_pred_bm: NDArray[np.float64] = -SO3.hat(pred_hat) @ R_WB.T
    J_pred_mw: NDArray[np.float64] = -R_WB.T @ R_BM @ SO3.hat(m_unit)

    J_wb: NDArray[np.float64] = SO3.hat(hat_meas) @ J_pred_wb
    J_bm: NDArray[np.float64] = SO3.hat(hat_meas) @ J_pred_bm
    J_mw: NDArray[np.float64] = SO3.hat(hat_meas) @ J_pred_mw

    return MagResidual(
        residual=residual,
        prediction=pred_hat,
        J_wb=J_wb.astype(np.float64),
        J_bm=J_bm.astype(np.float64),
        J_mw=J_mw.astype(np.float64),
    )
