################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for residual computations."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from oasis_control.localization.mounting.math_utils.quat import Quaternion
from oasis_control.localization.mounting.solver.residuals import AccelGravityResidual
from oasis_control.localization.mounting.solver.residuals import AccelResidual
from oasis_control.localization.mounting.solver.residuals import (
    accel_gravity_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.residuals import accel_residual
from oasis_control.localization.mounting.solver.residuals import (
    accel_residual_with_jacobians,
)
from oasis_control.localization.mounting.solver.residuals import mag_residual


def test_accel_residual_zero_when_aligned() -> None:
    """Check accel residual vanishes when aligned."""
    q_identity: NDArray[np.float64] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    g_W_unit: NDArray[np.float64] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    a_hat_meas: NDArray[np.float64] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    residual, prediction = accel_residual(a_hat_meas, q_identity, q_identity, g_W_unit)
    assert np.allclose(residual, np.zeros(3, dtype=np.float64), atol=1e-12)
    assert np.allclose(prediction, a_hat_meas, atol=1e-12)


def test_mag_residual_zero_when_aligned() -> None:
    """Check mag residual vanishes when aligned."""
    q_identity: NDArray[np.float64] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    m_W_unit: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    m_hat_meas: NDArray[np.float64] = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    residual, prediction = mag_residual(m_hat_meas, q_identity, q_identity, m_W_unit)
    assert np.allclose(residual, np.zeros(3, dtype=np.float64), atol=1e-12)
    assert np.allclose(prediction, m_hat_meas, atol=1e-12)


def test_small_angle_jacobian_sanity() -> None:
    """Check Jacobian matches finite differences for a small perturbation."""
    rng: np.random.Generator = np.random.default_rng(0)
    rotvec: NDArray[np.float64] = rng.normal(size=3).astype(np.float64)
    q_WB: NDArray[np.float64] = Quaternion.from_rotvec(rotvec).to_wxyz()
    q_BI: NDArray[np.float64] = Quaternion.from_rotvec(
        rng.normal(size=3).astype(np.float64)
    ).to_wxyz()
    g_W_unit: NDArray[np.float64] = rng.normal(size=3).astype(np.float64)
    g_W_unit = g_W_unit / np.linalg.norm(g_W_unit)
    a_meas: NDArray[np.float64] = rng.normal(size=3).astype(np.float64)
    a_meas = a_meas / np.linalg.norm(a_meas)

    residual0: NDArray[np.float64]
    residual0, _ = accel_residual(a_meas, q_WB, q_BI, g_W_unit)
    jac: AccelResidual = accel_residual_with_jacobians(
        a_meas,
        q_WB,
        q_BI,
        g_W_unit,
    )
    delta: NDArray[np.float64] = np.array([1e-6, -2e-6, 1e-6], dtype=np.float64)
    q_WB_perturbed: NDArray[np.float64] = (
        (Quaternion.from_rotvec(delta) * Quaternion(q_WB)).normalized().to_wxyz()
    )
    residual1: NDArray[np.float64]
    residual1, _ = accel_residual(a_meas, q_WB_perturbed, q_BI, g_W_unit)
    residual_lin: NDArray[np.float64] = residual0 + jac.J_wb @ delta
    assert np.allclose(residual1, residual_lin, atol=1e-6)


def test_accel_gravity_jacobians_match_finite_diff() -> None:
    """Check accel gravity Jacobians against finite differences."""
    rng: np.random.Generator = np.random.default_rng(4)
    q_WB: NDArray[np.float64] = Quaternion.from_rotvec(
        rng.normal(size=3).astype(np.float64)
    ).to_wxyz()
    q_BI: NDArray[np.float64] = Quaternion.from_rotvec(
        rng.normal(size=3).astype(np.float64)
    ).to_wxyz()
    g_W_unit: NDArray[np.float64] = rng.normal(size=3).astype(np.float64)
    g_W_unit = g_W_unit / np.linalg.norm(g_W_unit)
    g0_mps2: float = 9.80665
    accel_raw_mean: NDArray[np.float64] = -g0_mps2 * g_W_unit + rng.normal(
        scale=0.05, size=3
    ).astype(np.float64)
    b_a_mps2: NDArray[np.float64] = rng.normal(scale=0.02, size=3).astype(np.float64)
    A_a: NDArray[np.float64] = np.eye(3, dtype=np.float64) + (
        0.01 * rng.normal(size=(3, 3)).astype(np.float64)
    )
    eps: float = 1e-6

    def residuals(
        q_WB_wxyz: NDArray[np.float64],
        q_BI_wxyz: NDArray[np.float64],
        g_unit: NDArray[np.float64],
        b_a: NDArray[np.float64],
        A: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        result: AccelGravityResidual = accel_gravity_residual_with_jacobians(
            accel_raw_mean,
            b_a,
            A,
            q_WB_wxyz,
            q_BI_wxyz,
            g_unit,
            g0_mps2=g0_mps2,
        )
        return result.residual_dir, result.residual_mag

    base: AccelGravityResidual = accel_gravity_residual_with_jacobians(
        accel_raw_mean,
        b_a_mps2,
        A_a,
        q_WB,
        q_BI,
        g_W_unit,
        g0_mps2=g0_mps2,
    )

    J_wb_num: NDArray[np.float64] = np.zeros((3, 3), dtype=np.float64)
    J_bi_num: NDArray[np.float64] = np.zeros((3, 3), dtype=np.float64)
    J_gw_num: NDArray[np.float64] = np.zeros((3, 3), dtype=np.float64)
    J_ba_dir_num: NDArray[np.float64] = np.zeros((3, 3), dtype=np.float64)
    J_ba_mag_num: NDArray[np.float64] = np.zeros((1, 3), dtype=np.float64)
    J_Aa_dir_num: NDArray[np.float64] = np.zeros((3, 9), dtype=np.float64)
    J_Aa_mag_num: NDArray[np.float64] = np.zeros((1, 9), dtype=np.float64)

    for axis in range(3):
        delta_wb: NDArray[np.float64] = np.zeros(3, dtype=np.float64)
        delta_wb[axis] = eps
        q_plus_wb: NDArray[np.float64] = (
            (Quaternion.from_rotvec(delta_wb) * Quaternion(q_WB)).normalized().to_wxyz()
        )
        q_minus_wb: NDArray[np.float64] = (
            (Quaternion.from_rotvec(-delta_wb) * Quaternion(q_WB))
            .normalized()
            .to_wxyz()
        )
        res_plus_wb: NDArray[np.float64]
        res_minus_wb: NDArray[np.float64]
        res_plus_wb, _ = residuals(q_plus_wb, q_BI, g_W_unit, b_a_mps2, A_a)
        res_minus_wb, _ = residuals(q_minus_wb, q_BI, g_W_unit, b_a_mps2, A_a)
        J_wb_num[:, axis] = (res_plus_wb - res_minus_wb) / (2.0 * eps)

    for axis in range(3):
        delta_bi: NDArray[np.float64] = np.zeros(3, dtype=np.float64)
        delta_bi[axis] = eps
        q_plus_bi: NDArray[np.float64] = (
            (Quaternion.from_rotvec(delta_bi) * Quaternion(q_BI)).normalized().to_wxyz()
        )
        q_minus_bi: NDArray[np.float64] = (
            (Quaternion.from_rotvec(-delta_bi) * Quaternion(q_BI))
            .normalized()
            .to_wxyz()
        )
        res_plus_bi: NDArray[np.float64]
        res_minus_bi: NDArray[np.float64]
        res_plus_bi, _ = residuals(q_WB, q_plus_bi, g_W_unit, b_a_mps2, A_a)
        res_minus_bi, _ = residuals(q_WB, q_minus_bi, g_W_unit, b_a_mps2, A_a)
        J_bi_num[:, axis] = (res_plus_bi - res_minus_bi) / (2.0 * eps)

    for axis in range(3):
        delta_g: NDArray[np.float64] = np.zeros(3, dtype=np.float64)
        delta_g[axis] = eps
        g_plus: NDArray[np.float64] = (
            Quaternion.from_rotvec(delta_g).as_matrix() @ g_W_unit
        )
        g_minus: NDArray[np.float64] = (
            Quaternion.from_rotvec(-delta_g).as_matrix() @ g_W_unit
        )
        res_plus_g: NDArray[np.float64]
        res_minus_g: NDArray[np.float64]
        res_plus_g, _ = residuals(q_WB, q_BI, g_plus, b_a_mps2, A_a)
        res_minus_g, _ = residuals(q_WB, q_BI, g_minus, b_a_mps2, A_a)
        J_gw_num[:, axis] = (res_plus_g - res_minus_g) / (2.0 * eps)

    for axis in range(3):
        delta_ba: NDArray[np.float64] = np.zeros(3, dtype=np.float64)
        delta_ba[axis] = eps
        b_plus: NDArray[np.float64] = b_a_mps2 + delta_ba
        b_minus: NDArray[np.float64] = b_a_mps2 - delta_ba
        res_plus_dir_ba: NDArray[np.float64]
        res_plus_mag_ba: NDArray[np.float64]
        res_minus_dir_ba: NDArray[np.float64]
        res_minus_mag_ba: NDArray[np.float64]
        res_plus_dir_ba, res_plus_mag_ba = residuals(
            q_WB,
            q_BI,
            g_W_unit,
            b_plus,
            A_a,
        )
        res_minus_dir_ba, res_minus_mag_ba = residuals(
            q_WB,
            q_BI,
            g_W_unit,
            b_minus,
            A_a,
        )
        J_ba_dir_num[:, axis] = (res_plus_dir_ba - res_minus_dir_ba) / (2.0 * eps)
        J_ba_mag_num[:, axis] = (res_plus_mag_ba - res_minus_mag_ba) / (2.0 * eps)

    for idx in range(9):
        row: int = idx // 3
        col: int = idx % 3
        A_plus: NDArray[np.float64] = A_a.copy()
        A_minus: NDArray[np.float64] = A_a.copy()
        A_plus[row, col] += eps
        A_minus[row, col] -= eps
        res_plus_dir_aa: NDArray[np.float64]
        res_plus_mag_aa: NDArray[np.float64]
        res_minus_dir_aa: NDArray[np.float64]
        res_minus_mag_aa: NDArray[np.float64]
        res_plus_dir_aa, res_plus_mag_aa = residuals(
            q_WB,
            q_BI,
            g_W_unit,
            b_a_mps2,
            A_plus,
        )
        res_minus_dir_aa, res_minus_mag_aa = residuals(
            q_WB,
            q_BI,
            g_W_unit,
            b_a_mps2,
            A_minus,
        )
        J_Aa_dir_num[:, idx] = (res_plus_dir_aa - res_minus_dir_aa) / (2.0 * eps)
        J_Aa_mag_num[:, idx] = (res_plus_mag_aa - res_minus_mag_aa) / (2.0 * eps)

    np.testing.assert_allclose(J_wb_num, base.J_wb_dir, atol=2e-5, rtol=1e-4)
    np.testing.assert_allclose(J_bi_num, base.J_bi_dir, atol=2e-5, rtol=1e-4)
    np.testing.assert_allclose(J_gw_num, base.J_gw_dir, atol=2e-5, rtol=1e-4)
    np.testing.assert_allclose(J_ba_dir_num, base.J_ba_dir, atol=2e-5, rtol=1e-4)
    np.testing.assert_allclose(J_Aa_dir_num, base.J_Aa_dir, atol=2e-5, rtol=1e-4)
    np.testing.assert_allclose(J_ba_mag_num, base.J_ba_mag, atol=1e-6, rtol=1e-4)
    np.testing.assert_allclose(J_Aa_mag_num, base.J_Aa_mag, atol=1e-6, rtol=1e-4)
