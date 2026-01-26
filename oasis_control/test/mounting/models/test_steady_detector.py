################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the steady detector."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import SteadyParams
from oasis_control.localization.mounting.models.steady_detector import SteadyDetector
from oasis_control.localization.mounting.models.steady_detector import (
    SteadyDetectorError,
)
from oasis_control.localization.mounting.mounting_types import MagPacket
from oasis_control.localization.mounting.mounting_types import SteadySegment


def _steady_params() -> MountingParams:
    """Create mounting params configured for steady detection tests."""
    base_params: MountingParams = MountingParams.defaults()
    steady: SteadyParams = replace(
        base_params.steady,
        steady_sec=1.0,
        omega_mean_thresh=0.1,
        omega_cov_thresh=0.01,
        a_cov_thresh=0.01,
        a_norm_min=9.0,
        a_norm_max=10.5,
        window_type="sliding",
    )
    return replace(base_params, steady=steady)


def _push_sample(
    detector: SteadyDetector,
    *,
    t_ns: int,
    omega: np.ndarray,
    accel: np.ndarray,
    mag: MagPacket | None = None,
) -> None:
    """Push a sample and ignore the result."""
    detector.push(
        t_ns=t_ns,
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
        mag=mag,
    )


def test_requires_full_window_span() -> None:
    """Ensure the window spans steady_sec before steady checks apply."""
    params: MountingParams = _steady_params()
    detector: SteadyDetector = SteadyDetector(params)
    omega: np.ndarray = np.zeros(3, dtype=np.float64)
    accel: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)

    segment: SteadySegment | None = detector.push(
        t_ns=0,
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is None

    segment = detector.push(
        t_ns=int(0.8e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is None


def test_sliding_window_emits_once() -> None:
    """Ensure the detector emits once after steady duration."""
    params: MountingParams = _steady_params()
    detector: SteadyDetector = SteadyDetector(params)
    omega: np.ndarray = np.zeros(3, dtype=np.float64)
    accel: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)

    segment: SteadySegment | None = detector.push(
        t_ns=0,
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is None

    segment = detector.push(
        t_ns=int(0.5e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is None

    segment = detector.push(
        t_ns=int(1.0e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is None

    segment_again: SteadySegment | None = detector.push(
        t_ns=int(1.5e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment_again is None

    segment = detector.push(
        t_ns=int(2.0e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is not None


def test_exit_and_reenter_steady() -> None:
    """Ensure re-entering steady emits a new segment."""
    params: MountingParams = _steady_params()
    detector: SteadyDetector = SteadyDetector(params)
    omega_ok: np.ndarray = np.zeros(3, dtype=np.float64)
    accel_ok: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    omega_bad: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    _push_sample(detector, t_ns=0, omega=omega_ok, accel=accel_ok)
    _push_sample(detector, t_ns=int(1.0e9), omega=omega_ok, accel=accel_ok)
    segment: SteadySegment | None = detector.push(
        t_ns=int(2.0e9),
        omega_corr_rads=omega_ok,
        a_corr_mps2=accel_ok,
        imu_frame_id="imu",
    )
    assert segment is not None

    segment = detector.push(
        t_ns=int(2.5e9),
        omega_corr_rads=omega_bad,
        a_corr_mps2=accel_ok,
        imu_frame_id="imu",
    )
    assert segment is None

    _push_sample(detector, t_ns=int(3.0e9), omega=omega_ok, accel=accel_ok)
    _push_sample(detector, t_ns=int(4.0e9), omega=omega_ok, accel=accel_ok)
    segment = detector.push(
        t_ns=int(5.0e9),
        omega_corr_rads=omega_ok,
        a_corr_mps2=accel_ok,
        imu_frame_id="imu",
    )
    assert segment is not None


def test_threshold_failure_prevents_emission() -> None:
    """Ensure threshold violations block emission."""
    params: MountingParams = _steady_params()
    detector: SteadyDetector = SteadyDetector(params)
    omega: np.ndarray = np.array([0.2, 0.0, 0.0], dtype=np.float64)
    accel: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)

    _push_sample(detector, t_ns=0, omega=omega, accel=accel)
    segment = detector.push(
        t_ns=int(1.0e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    assert segment is None


def test_mag_samples_included() -> None:
    """Ensure magnetometer samples are aggregated when available."""
    params: MountingParams = _steady_params()
    detector: SteadyDetector = SteadyDetector(params)
    omega: np.ndarray = np.zeros(3, dtype=np.float64)
    accel: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    mag_cov: np.ndarray = np.eye(3, dtype=np.float64) * 0.01

    mag0: MagPacket = MagPacket(
        t_meas_ns=int(1.0e9),
        frame_id="mag",
        m_raw_T=np.array([0.1, 0.0, 0.0], dtype=np.float64),
        cov_m_raw_T2=mag_cov,
    )
    mag1: MagPacket = MagPacket(
        t_meas_ns=int(2.0e9),
        frame_id="mag",
        m_raw_T=np.array([0.2, 0.0, 0.0], dtype=np.float64),
        cov_m_raw_T2=mag_cov,
    )

    detector.push(
        t_ns=0,
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
    )
    detector.push(
        t_ns=int(1.0e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
        mag=mag0,
    )
    segment = detector.push(
        t_ns=int(2.0e9),
        omega_corr_rads=omega,
        a_corr_mps2=accel,
        imu_frame_id="imu",
        mag=mag1,
    )
    assert segment is not None
    assert segment.mag_frame_id == "mag"
    assert segment.m_mean_T is not None
    np.testing.assert_allclose(segment.m_mean_T, np.array([0.15, 0.0, 0.0]))


def test_non_monotonic_time_raises() -> None:
    """Ensure non-monotonic time raises a detector error."""
    params: MountingParams = _steady_params()
    detector: SteadyDetector = SteadyDetector(params)
    omega: np.ndarray = np.zeros(3, dtype=np.float64)
    accel: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)

    _push_sample(detector, t_ns=0, omega=omega, accel=accel)
    with pytest.raises(SteadyDetectorError):
        _push_sample(detector, t_ns=0, omega=omega, accel=accel)
