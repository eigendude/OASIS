################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Simulation helpers for Falcon mounting integration tests."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Sequence
from typing import cast

import numpy as np
import yaml

from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.mounting_types import MagPacket


# s, sample interval for a 50 Hz stream
SAMPLE_DT_SEC: float = 0.02

# m/s^2, standard gravity magnitude
GRAVITY_MPS2: float = 9.80665

# unitless, gravity magnitude bias scale for +0.1 g
GRAVITY_BIAS_SCALE: float = 1.1

# tesla, nominal magnetic field magnitude used in simulation
MAG_FIELD_T: float = 50e-6

# unitless, Cholesky stabilization for covariance noise draws
NOISE_EPS: float = 1e-18


@dataclass(frozen=True)
class ImuCalibrationData:
    """Calibration data loaded from an IMU YAML resource.

    Attributes:
        prior: Calibration prior for the IMU packet
        cov_omega_raw: Raw gyro covariance in (rad/s)^2
        cov_a_raw: Raw accel covariance in (m/s^2)^2
        cov_omega_corr: Corrected gyro covariance in (rad/s)^2
        cov_a_corr: Corrected accel covariance in (m/s^2)^2
    """

    prior: ImuCalibrationPrior
    cov_omega_raw: np.ndarray
    cov_a_raw: np.ndarray
    cov_omega_corr: np.ndarray
    cov_a_corr: np.ndarray


@dataclass(frozen=True)
class MotionSegment:
    """Segment of simulated motion with a target gravity direction.

    Attributes:
        duration_sec: Duration of the segment in seconds
        gravity_start_unit: Gravity direction at the start of the segment
        gravity_end_unit: Gravity direction at the end of the segment
        is_stationary: True when the segment should pass steady gating
    """

    duration_sec: float
    gravity_start_unit: np.ndarray
    gravity_end_unit: np.ndarray
    is_stationary: bool


@dataclass(frozen=True)
class FalconSample:
    """Simulated IMU and magnetometer sample.

    Attributes:
        t_ns: Sample timestamp in nanoseconds
        imu_packet: Simulated IMU packet
        mag_packet: Simulated magnetometer packet when available
        gravity_unit: True gravity direction in the IMU frame
        expected_stationary: True when the sample should be steady
    """

    t_ns: int
    imu_packet: ImuPacket
    mag_packet: MagPacket | None
    gravity_unit: np.ndarray
    expected_stationary: bool


def resource_path(filename: str) -> Path:
    """Return a resource path relative to the integration test directory."""
    base_dir: Path = Path(__file__).resolve().parent
    resources_dir: Path = base_dir.parent / "resources"
    return resources_dir / filename


def load_imu_calibration(path: Path, *, frame_id: str) -> ImuCalibrationData:
    """Load IMU calibration prior and noise covariances from YAML."""
    data: dict[str, Any] = _load_yaml(path)
    calib: dict[str, Any] = cast(dict[str, Any], data["calib"])
    noise: dict[str, Any] = cast(dict[str, Any], data["measurement_noise"])

    b_a_mps2: np.ndarray = np.array(calib["accel_bias_mps2"], dtype=np.float64)
    A_a_row: np.ndarray = np.array(calib["accel_A_row_major"], dtype=np.float64)
    A_a: np.ndarray = A_a_row.reshape(3, 3)
    b_g_rads: np.ndarray = np.array(calib["gyro_bias_rads"], dtype=np.float64)

    cov_a_raw: np.ndarray = np.array(
        noise["accel_cov_raw_mps2_2"], dtype=np.float64
    ).reshape(3, 3)
    cov_omega_raw: np.ndarray = np.array(
        noise["gyro_cov_raw_rads2_2"], dtype=np.float64
    ).reshape(3, 3)

    cov_a_corr: np.ndarray = np.array(
        noise["accel_cov_corrected_mps2_2"], dtype=np.float64
    ).reshape(3, 3)
    cov_omega_corr: np.ndarray = np.array(
        noise["gyro_cov_corrected_rads2_2"], dtype=np.float64
    ).reshape(3, 3)

    prior: ImuCalibrationPrior = ImuCalibrationPrior(
        valid=True,
        frame_id=frame_id,
        b_a_mps2=b_a_mps2,
        A_a=A_a,
        b_g_rads=b_g_rads,
        cov_a_params=None,
        cov_b_g=None,
    )
    return ImuCalibrationData(
        prior=prior,
        cov_omega_raw=cov_omega_raw,
        cov_a_raw=cov_a_raw,
        cov_omega_corr=cov_omega_corr,
        cov_a_corr=cov_a_corr,
    )


def load_mag_covariance(path: Path) -> np.ndarray:
    """Load a 3x3 magnetometer covariance matrix from YAML."""
    data: dict[str, Any] = _load_yaml(path)
    cov_entries: np.ndarray = np.array(data["covariance_t2"], dtype=np.float64)
    return cov_entries.reshape(3, 3)


def pose_sequence(
    *,
    gravity_units: Sequence[np.ndarray],
    hold_sec: float,
    move_sec: float,
) -> list[MotionSegment]:
    """Build a hold/move pose sequence for a list of gravity directions."""
    if not gravity_units:
        raise ValueError("gravity_units must be non-empty")
    if hold_sec <= 0.0:
        raise ValueError("hold_sec must be positive")
    if move_sec <= 0.0:
        raise ValueError("move_sec must be positive")

    segments: list[MotionSegment] = []
    idx: int
    for idx, gravity_unit in enumerate(gravity_units):
        unit_start: np.ndarray = _unit_vector(gravity_unit)
        segments.append(
            MotionSegment(
                duration_sec=hold_sec,
                gravity_start_unit=unit_start,
                gravity_end_unit=unit_start,
                is_stationary=True,
            )
        )
        if idx + 1 >= len(gravity_units):
            continue
        unit_end: np.ndarray = _unit_vector(gravity_units[idx + 1])
        segments.append(
            MotionSegment(
                duration_sec=move_sec,
                gravity_start_unit=unit_start,
                gravity_end_unit=unit_end,
                is_stationary=False,
            )
        )
    return segments


def simulate_falcon_sequence(
    *,
    segments: Sequence[MotionSegment],
    imu_calib: ImuCalibrationData,
    mag_cov: np.ndarray,
    imu_frame_id: str,
    mag_frame_id: str,
    dt_sec: float = SAMPLE_DT_SEC,
    seed: int = 7,
    include_mag: bool = True,
) -> list[FalconSample]:
    """Generate a deterministic sequence of IMU and mag samples."""
    if dt_sec <= 0.0:
        raise ValueError("dt_sec must be positive")
    if not segments:
        raise ValueError("segments must be non-empty")

    rng: np.random.Generator = np.random.default_rng(seed)
    A_inv: np.ndarray = _invert_accel_matrix(imu_calib.prior.A_a)

    mag_unit: np.ndarray = _unit_vector(np.array([0.4, 0.2, -0.1], dtype=np.float64))
    mag_base: np.ndarray = MAG_FIELD_T * mag_unit

    samples: list[FalconSample] = []
    t_ns: int = 0
    segment: MotionSegment
    for segment in segments:
        steps: int = int(round(segment.duration_sec / dt_sec))
        if steps <= 0:
            raise ValueError("segment duration too short for dt_sec")
        step_idx: int
        for step_idx in range(steps):
            t_sec: float = float(t_ns) / 1e9
            alpha: float = _segment_alpha(step_idx, steps)
            gravity_unit: np.ndarray = _interpolate_unit(
                segment.gravity_start_unit,
                segment.gravity_end_unit,
                alpha,
            )

            accel_corr: np.ndarray = _accel_signal(
                t_sec=t_sec,
                gravity_unit=gravity_unit,
                stationary=segment.is_stationary,
                cov=imu_calib.cov_a_corr,
                rng=rng,
            )
            omega_corr: np.ndarray = _omega_signal(
                t_sec=t_sec,
                stationary=segment.is_stationary,
                cov=imu_calib.cov_omega_corr,
                rng=rng,
            )

            accel_raw: np.ndarray = A_inv @ accel_corr + imu_calib.prior.b_a_mps2
            omega_raw: np.ndarray = omega_corr + imu_calib.prior.b_g_rads

            imu_packet: ImuPacket = ImuPacket(
                t_meas_ns=t_ns,
                frame_id=imu_frame_id,
                omega_raw_rads=omega_raw,
                cov_omega_raw=imu_calib.cov_omega_raw,
                a_raw_mps2=accel_raw,
                cov_a_raw=imu_calib.cov_a_raw,
                calibration=imu_calib.prior,
            )

            mag_packet: MagPacket | None
            if include_mag:
                mag_noise: np.ndarray = _draw_correlated_noise(
                    mag_cov, rng=rng, eps=NOISE_EPS
                )
                mag_raw: np.ndarray = mag_base + mag_noise
                mag_packet = MagPacket(
                    t_meas_ns=t_ns,
                    frame_id=mag_frame_id,
                    m_raw_T=mag_raw,
                    cov_m_raw_T2=mag_cov,
                )
            else:
                mag_packet = None

            samples.append(
                FalconSample(
                    t_ns=t_ns,
                    imu_packet=imu_packet,
                    mag_packet=mag_packet,
                    gravity_unit=gravity_unit,
                    expected_stationary=segment.is_stationary,
                )
            )
            t_ns += int(round(dt_sec * 1e9))
    return samples


def _load_yaml(path: Path) -> dict[str, Any]:
    """Load a YAML file into a dictionary."""
    text: str = path.read_text(encoding="utf-8")
    data: Any = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return cast(dict[str, Any], data)


def _unit_vector(vector: np.ndarray) -> np.ndarray:
    """Return a unit-length vector from an array-like input."""
    array: np.ndarray = np.asarray(vector, dtype=np.float64)
    if array.shape != (3,):
        raise ValueError("vector must have shape (3,)")
    if not np.all(np.isfinite(array)):
        raise ValueError("vector must be finite")
    norm: float = float(np.linalg.norm(array))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("vector norm must be positive")
    return array / norm


def _segment_alpha(step_idx: int, steps: int) -> float:
    """Return interpolation alpha for a step within a segment."""
    if steps <= 1:
        return 0.0
    return float(step_idx) / float(steps - 1)


def _interpolate_unit(
    start: np.ndarray,
    end: np.ndarray,
    alpha: float,
) -> np.ndarray:
    """Linearly interpolate two unit vectors and renormalize."""
    blended: np.ndarray = (1.0 - alpha) * start + alpha * end
    return _unit_vector(blended)


def _invert_accel_matrix(A_a: np.ndarray) -> np.ndarray:
    """Return a stable inverse for an accelerometer calibration matrix."""
    matrix: np.ndarray = np.asarray(A_a, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("A_a must have shape (3, 3)")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("A_a must be finite")
    cond: float = float(np.linalg.cond(matrix))
    if not np.isfinite(cond) or cond > 1e6:
        return np.linalg.pinv(matrix)
    return np.linalg.inv(matrix)


def _draw_correlated_noise(
    cov: np.ndarray,
    *,
    rng: np.random.Generator,
    eps: float,
) -> np.ndarray:
    """Return correlated Gaussian noise for a 3x3 covariance."""
    cov_array: np.ndarray = np.asarray(cov, dtype=np.float64)
    if cov_array.shape != (3, 3):
        raise ValueError("cov must have shape (3, 3)")
    if not np.all(np.isfinite(cov_array)):
        raise ValueError("cov must be finite")
    cov_stable: np.ndarray = cov_array + np.eye(3, dtype=np.float64) * eps
    try:
        chol: np.ndarray = np.linalg.cholesky(cov_stable)
    except np.linalg.LinAlgError:
        eigvals: np.ndarray
        eigvecs: np.ndarray
        eigvals, eigvecs = np.linalg.eigh(cov_stable)
        eigvals = np.clip(eigvals, a_min=0.0, a_max=None)
        chol = eigvecs @ np.diag(np.sqrt(eigvals))
    noise: np.ndarray = rng.standard_normal(3)
    return chol @ noise


def _omega_signal(
    *,
    t_sec: float,
    stationary: bool,
    cov: np.ndarray,
    rng: np.random.Generator,
) -> np.ndarray:
    """Return a corrected gyro signal for the selected motion regime."""
    omega_bias: np.ndarray
    omega_drift: np.ndarray
    if stationary:
        # rad/s, steady bias for struggle motion during holds
        omega_bias = np.array([0.002, -0.003, 0.001], dtype=np.float64)

        # rad/s, low-frequency drift amplitude for struggle motion
        omega_drift = 0.008 * np.array(
            [
                np.sin(2.0 * np.pi * 0.15 * t_sec),
                np.cos(2.0 * np.pi * 0.12 * t_sec),
                np.sin(2.0 * np.pi * 0.07 * t_sec),
            ],
            dtype=np.float64,
        )
    else:
        # rad/s, bias during deliberate motion between poses
        omega_bias = np.array([0.12, -0.18, 0.08], dtype=np.float64)

        # rad/s, high-frequency drift amplitude during motion segments
        omega_drift = 0.15 * np.array(
            [
                np.sin(2.0 * np.pi * 0.7 * t_sec),
                np.cos(2.0 * np.pi * 0.5 * t_sec),
                np.sin(2.0 * np.pi * 0.4 * t_sec),
            ],
            dtype=np.float64,
        )
    omega_noise: np.ndarray = _draw_correlated_noise(cov, rng=rng, eps=NOISE_EPS)
    return omega_bias + omega_drift + omega_noise


def _accel_signal(
    *,
    t_sec: float,
    gravity_unit: np.ndarray,
    stationary: bool,
    cov: np.ndarray,
    rng: np.random.Generator,
) -> np.ndarray:
    """Return a corrected accelerometer signal for the motion regime."""
    # m/s^2, biased gravity magnitude to include +0.1 g bias
    g_mps2: float = GRAVITY_MPS2 * GRAVITY_BIAS_SCALE
    gravity_vec: np.ndarray = -g_mps2 * gravity_unit
    accel_drift: np.ndarray
    if stationary:
        # m/s^2, low-frequency drift amplitude during steady holds
        accel_drift = 0.05 * np.array(
            [
                np.sin(2.0 * np.pi * 0.05 * t_sec),
                np.cos(2.0 * np.pi * 0.07 * t_sec),
                np.sin(2.0 * np.pi * 0.03 * t_sec),
            ],
            dtype=np.float64,
        )
    else:
        # m/s^2, disturbance amplitude during motion segments
        accel_drift = 1.2 * np.array(
            [
                np.sin(2.0 * np.pi * 0.6 * t_sec),
                np.cos(2.0 * np.pi * 0.9 * t_sec),
                np.sin(2.0 * np.pi * 0.4 * t_sec),
            ],
            dtype=np.float64,
        )
    accel_noise: np.ndarray = _draw_correlated_noise(cov, rng=rng, eps=NOISE_EPS)
    return gravity_vec + accel_drift + accel_noise
