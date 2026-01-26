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
from oasis_control.localization.mounting.solver.residuals import AccelResidual
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
