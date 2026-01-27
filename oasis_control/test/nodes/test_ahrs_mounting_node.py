################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for AHRS mounting node helpers."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import replace

import numpy as np
import pytest

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.diagnostics.imu_pair_tracker import (
    ImuPairTracker,
)
from oasis_control.localization.mounting.math_utils.validation import (
    CovarianceValidationError,
)
from oasis_control.localization.mounting.math_utils.validation import (
    normalize_quaternion_wxyz,
)
from oasis_control.localization.mounting.math_utils.validation import reshape_covariance
from oasis_control.localization.mounting.math_utils.validation import reshape_matrix
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.ros.imu_packet_builder import (
    build_imu_packet as _build_imu_packet,
)


@dataclass
class FakeStamp:
    """Minimal timestamp container for IMU messages"""

    sec: int
    nanosec: int


@dataclass
class FakeHeader:
    """Minimal header container for IMU messages"""

    stamp: FakeStamp
    frame_id: str


@dataclass
class FakeVector3:
    """Minimal vector container for IMU messages"""

    x: float
    y: float
    z: float


@dataclass
class FakeImuMsg:
    """Minimal IMU message container for helper tests"""

    header: FakeHeader
    angular_velocity: FakeVector3
    linear_acceleration: FakeVector3
    angular_velocity_covariance: list[float]
    linear_acceleration_covariance: list[float]


@dataclass
class FakeImuCalibrationMsg:
    """Minimal IMU calibration container for helper tests"""

    header: FakeHeader
    valid: bool
    accel_bias: FakeVector3
    accel_a: list[float]
    accel_param_cov: list[float]
    gyro_bias: FakeVector3
    gyro_bias_cov: list[float]


def test_reshape_matrix_accepts_negative_diagonal() -> None:
    """Ensure accel calibration matrices allow negative diagonals."""
    values: list[float] = [
        -1.0,
        0.1,
        0.2,
        0.0,
        -0.5,
        0.3,
        0.0,
        0.0,
        -0.2,
    ]
    matrix: np.ndarray = reshape_matrix(values, (3, 3), "accel_a")
    assert matrix.shape == (3, 3)
    assert matrix[0, 0] < 0.0


def test_reshape_covariance_accepts_unknown_all_minus_one() -> None:
    """Ensure all -1 covariances are substituted."""
    values: list[float] = [-1.0] * 9
    fallback: np.ndarray = np.diag([0.1, 0.2, 0.3])
    matrix: np.ndarray = reshape_covariance(
        values,
        (3, 3),
        "covariance",
        fallback=fallback,
    )
    assert np.allclose(matrix, fallback)


def test_reshape_covariance_accepts_unknown_diag_minus_one() -> None:
    """Ensure -1 diagonal covariances are substituted."""
    values: list[float] = [
        -1.0,
        0.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        -1.0,
    ]
    fallback: np.ndarray = np.diag([0.4, 0.5, 0.6])
    matrix: np.ndarray = reshape_covariance(
        values,
        (3, 3),
        "covariance",
        fallback=fallback,
    )
    assert np.allclose(matrix, fallback)


def test_reshape_covariance_rejects_unknown_without_fallback() -> None:
    """Ensure unknown covariances still raise without fallback."""
    values: list[float] = [-1.0] * 9
    with pytest.raises(CovarianceValidationError):
        reshape_covariance(values, (3, 3), "covariance")


def test_reshape_covariance_rejects_negative_diagonal() -> None:
    """Ensure negative diagonal entries are rejected."""
    values: list[float] = [
        -0.1,
        0.0,
        0.0,
        0.0,
        0.2,
        0.0,
        0.0,
        0.0,
        0.3,
    ]
    with pytest.raises(ValueError):
        reshape_covariance(values, (3, 3), "covariance")


def test_reshape_covariance_rejects_nan_values() -> None:
    """Ensure NaN covariances are rejected."""
    values: list[float] = [
        0.1,
        0.0,
        0.0,
        0.0,
        float("nan"),
        0.0,
        0.0,
        0.0,
        0.3,
    ]
    with pytest.raises(ValueError):
        reshape_covariance(values, (3, 3), "covariance")


def test_imu_pair_tracker_rejects_bad_cov_without_unpaired_drop() -> None:
    """Ensure covariance rejection increments bad cov, not unpaired drops."""
    tracker: ImuPairTracker = ImuPairTracker(warmup_sec=0.0)
    t_ns: int = 1_000_000
    tracker.record_raw(t_ns)
    tracker.record_calibration(t_ns)
    tracker.record_pair(t_ns)
    tracker.reject_bad_cov()
    assert tracker.imu_pairs_rejected_bad_cov == 1
    assert tracker.unpaired_raw(t_ns) == 0


def test_imu_pair_tracker_counts_unpaired_raw() -> None:
    """Ensure unpaired raw messages are counted after warmup."""
    tracker: ImuPairTracker = ImuPairTracker(warmup_sec=0.0)
    t_ns: int = 2_000_000
    tracker.record_raw(t_ns)
    assert tracker.unpaired_raw(t_ns) == 1


def test_quaternion_normalization() -> None:
    """Ensure quaternion inputs are normalized."""
    wxyz: list[float] = [2.0, 0.0, 0.0, 0.0]
    normalized: np.ndarray = normalize_quaternion_wxyz(wxyz)
    assert np.isclose(normalized[0], 1.0)
    assert np.isclose(normalized[1], 0.0)
    assert np.isclose(normalized[2], 0.0)
    assert np.isclose(normalized[3], 0.0)


def test_build_imu_packet_uses_param_defaults_for_unknown_covariance() -> None:
    """Ensure IMU covariance fallbacks ignore calibration parameters."""
    omega_diag: np.ndarray = np.array([1.1e-05, 2.2e-05, 3.3e-05], dtype=np.float64)
    accel_diag: np.ndarray = np.array([4.4e-03, 5.5e-03, 6.6e-03], dtype=np.float64)
    params: MountingParams = MountingParams.defaults()
    params = replace(
        params,
        imu=replace(params.imu, omega_cov_diag=omega_diag, accel_cov_diag=accel_diag),
    )

    imu_msg: FakeImuMsg = FakeImuMsg(
        header=FakeHeader(stamp=FakeStamp(sec=1, nanosec=2), frame_id="imu"),
        angular_velocity=FakeVector3(x=0.1, y=0.2, z=0.3),
        linear_acceleration=FakeVector3(x=1.0, y=2.0, z=3.0),
        angular_velocity_covariance=[-1.0] * 9,
        linear_acceleration_covariance=[-1.0] * 9,
    )

    cal_msg: FakeImuCalibrationMsg = FakeImuCalibrationMsg(
        header=FakeHeader(stamp=FakeStamp(sec=1, nanosec=2), frame_id="imu"),
        valid=True,
        accel_bias=FakeVector3(x=0.1, y=0.2, z=0.3),
        accel_a=np.eye(3, dtype=np.float64).reshape(-1).tolist(),
        accel_param_cov=np.eye(12, dtype=np.float64).reshape(-1).tolist(),
        gyro_bias=FakeVector3(x=0.4, y=0.5, z=0.6),
        gyro_bias_cov=np.eye(3, dtype=np.float64).reshape(-1).tolist(),
    )

    packet: ImuPacket = _build_imu_packet(imu_msg, cal_msg, params=params)

    assert np.allclose(packet.cov_omega_raw, np.diag(omega_diag))
    assert np.allclose(packet.cov_a_raw, np.diag(accel_diag))
