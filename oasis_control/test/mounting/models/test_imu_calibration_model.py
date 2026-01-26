################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the IMU calibration model."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.models.imu_calibration_model import (
    ImuCalibrationModel,
)
from oasis_control.localization.mounting.models.imu_calibration_model import (
    ImuCalibrationModelError,
)
from oasis_control.localization.mounting.models.imu_calibration_model import (
    ImuNuisanceState,
)
from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior


def test_state_from_prior_valid() -> None:
    """Verify that a valid prior maps into a nuisance state."""
    prior: ImuCalibrationPrior = ImuCalibrationPrior(
        valid=True,
        frame_id="imu",
        b_a_mps2=np.array([0.1, 0.2, 0.3], dtype=np.float64),
        A_a=np.array(
            [[1.0, 0.0, 0.0], [0.0, 1.1, 0.0], [0.0, 0.0, 0.9]],
            dtype=np.float64,
        ),
        b_g_rads=np.array([0.01, -0.02, 0.03], dtype=np.float64),
        cov_a_params=None,
        cov_b_g=None,
    )

    state: ImuNuisanceState = ImuCalibrationModel.state_from_prior(prior)

    np.testing.assert_allclose(state.b_a_mps2, prior.b_a_mps2)
    np.testing.assert_allclose(state.A_a, prior.A_a)
    np.testing.assert_allclose(state.b_g_rads, prior.b_g_rads)


def test_state_from_prior_invalid_returns_default() -> None:
    """Verify that an invalid prior returns the default state."""
    prior: ImuCalibrationPrior = ImuCalibrationPrior(
        valid=False,
        frame_id="imu",
        b_a_mps2=np.zeros(3, dtype=np.float64),
        A_a=np.eye(3, dtype=np.float64),
        b_g_rads=np.zeros(3, dtype=np.float64),
        cov_a_params=None,
        cov_b_g=None,
    )

    state: ImuNuisanceState = ImuCalibrationModel.state_from_prior(prior)

    np.testing.assert_allclose(state.b_a_mps2, np.zeros(3, dtype=np.float64))
    np.testing.assert_allclose(state.A_a, np.eye(3, dtype=np.float64))
    np.testing.assert_allclose(state.b_g_rads, np.zeros(3, dtype=np.float64))


def test_nuisance_state_validation() -> None:
    """Ensure that invalid nuisance states raise errors."""
    with pytest.raises(ImuCalibrationModelError):
        ImuNuisanceState(
            b_a_mps2=np.zeros(2, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
        )
    with pytest.raises(ImuCalibrationModelError):
        ImuNuisanceState(
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.array([np.nan, 0.0, 0.0], dtype=np.float64),
        )


def test_correction_formulas() -> None:
    """Verify accelerometer and gyro correction formulas."""
    state: ImuNuisanceState = ImuNuisanceState(
        b_a_mps2=np.array([0.5, -0.5, 1.0], dtype=np.float64),
        A_a=np.array(
            [[1.0, 0.0, 0.0], [0.1, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        ),
        b_g_rads=np.array([0.01, 0.02, 0.03], dtype=np.float64),
    )
    a_raw: np.ndarray = np.array([1.5, -0.5, 2.0], dtype=np.float64)
    omega_raw: np.ndarray = np.array([0.1, 0.2, 0.3], dtype=np.float64)

    a_corr: np.ndarray = state.correct_accel(a_raw)
    omega_corr: np.ndarray = state.correct_gyro(omega_raw)

    expected_a: np.ndarray = state.A_a @ (a_raw - state.b_a_mps2)
    expected_omega: np.ndarray = omega_raw - state.b_g_rads

    np.testing.assert_allclose(a_corr, expected_a)
    np.testing.assert_allclose(omega_corr, expected_omega)
