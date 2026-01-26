################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for IMU packet types."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.mounting.mounting_types.imu_packet import (
    ImuCalibrationPrior,
)
from oasis_control.localization.mounting.mounting_types.imu_packet import ImuPacket


def _make_prior() -> ImuCalibrationPrior:
    """Create a valid ImuCalibrationPrior for tests."""
    b_a_mps2: np.ndarray = np.array([0.1, -0.2, 0.05], dtype=np.float64)
    A_a: np.ndarray = np.eye(3, dtype=np.float64)
    b_g_rads: np.ndarray = np.array([0.01, 0.02, -0.01], dtype=np.float64)
    cov_a_params: np.ndarray = np.eye(12, dtype=np.float64)
    cov_b_g: np.ndarray = np.eye(3, dtype=np.float64)
    return ImuCalibrationPrior(
        valid=True,
        frame_id="imu",
        b_a_mps2=b_a_mps2,
        A_a=A_a,
        b_g_rads=b_g_rads,
        cov_a_params=cov_a_params,
        cov_b_g=cov_b_g,
    )


def test_imu_packet_has_calibration() -> None:
    """Ensure calibration presence is reported correctly."""
    prior: ImuCalibrationPrior = _make_prior()
    packet_with: ImuPacket = ImuPacket(
        t_meas_ns=100,
        frame_id="imu",
        omega_raw_rads=np.array([0.0, 0.1, 0.2], dtype=np.float64),
        cov_omega_raw=np.eye(3, dtype=np.float64),
        a_raw_mps2=np.array([0.0, 0.0, 9.81], dtype=np.float64),
        cov_a_raw=np.eye(3, dtype=np.float64),
        calibration=prior,
    )
    packet_without: ImuPacket = ImuPacket(
        t_meas_ns=101,
        frame_id="imu",
        omega_raw_rads=np.array([0.0, 0.1, 0.2], dtype=np.float64),
        cov_omega_raw=np.eye(3, dtype=np.float64),
        a_raw_mps2=np.array([0.0, 0.0, 9.81], dtype=np.float64),
        cov_a_raw=np.eye(3, dtype=np.float64),
        calibration=None,
    )
    assert packet_with.has_calibration()
    assert not packet_without.has_calibration()


def test_imu_packet_shape_validation() -> None:
    """Ensure shape validation rejects invalid arrays."""
    with pytest.raises(ValueError):
        ImuPacket(
            t_meas_ns=0,
            frame_id="imu",
            omega_raw_rads=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            cov_omega_raw=np.eye(2, dtype=np.float64),
            a_raw_mps2=np.array([0.0, 0.0, 9.81], dtype=np.float64),
            cov_a_raw=np.eye(3, dtype=np.float64),
            calibration=None,
        )


def test_imu_calibration_prior_finite_validation() -> None:
    """Ensure finite validation rejects NaN values."""
    A_a: np.ndarray = np.eye(3, dtype=np.float64)
    A_a[0, 0] = np.nan
    with pytest.raises(ValueError):
        ImuCalibrationPrior(
            valid=True,
            frame_id="imu",
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=A_a,
            b_g_rads=np.zeros(3, dtype=np.float64),
            cov_a_params=None,
            cov_b_g=None,
        )


def test_imu_calibration_prior_shape_validation() -> None:
    """Ensure covariance shapes are validated."""
    with pytest.raises(ValueError):
        ImuCalibrationPrior(
            valid=True,
            frame_id="imu",
            b_a_mps2=np.zeros(3, dtype=np.float64),
            A_a=np.eye(3, dtype=np.float64),
            b_g_rads=np.zeros(3, dtype=np.float64),
            cov_a_params=np.eye(3, dtype=np.float64),
            cov_b_g=None,
        )
