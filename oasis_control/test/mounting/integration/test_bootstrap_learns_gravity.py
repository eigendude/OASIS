################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Integration test for bootstrapping gravity direction in the mounting pipeline."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any
from typing import cast

import numpy as np
import yaml

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.mounting_types import MagPacket
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipeline,
)
from oasis_control.localization.mounting.storage.yaml_format import FlagsYaml


def _load_yaml(path: Path) -> dict[str, Any]:
    """Load a YAML file into a dictionary."""
    text: str = path.read_text(encoding="utf-8")
    data: Any = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError(f"Expected mapping in {path}")
    return cast(dict[str, Any], data)


def _load_imu_calibration(path: Path, *, frame_id: str) -> tuple[
    ImuCalibrationPrior,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """Load IMU calibration prior and measurement covariances."""
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
    return prior, cov_omega_raw, cov_a_raw, cov_omega_corr, cov_a_corr


def _load_mag_covariance(path: Path) -> np.ndarray:
    """Load a 3x3 magnetometer covariance matrix from YAML."""
    data: dict[str, Any] = _load_yaml(path)
    cov_entries: np.ndarray = np.array(data["covariance_t2"], dtype=np.float64)
    return cov_entries.reshape(3, 3)


def _cholesky_noise(
    cov: np.ndarray, *, rng: np.random.Generator, eps: float
) -> np.ndarray:
    """Draw correlated noise for a covariance with a small stabilization term."""
    cov_stable: np.ndarray = cov + np.eye(3, dtype=np.float64) * eps
    L: np.ndarray = np.linalg.cholesky(cov_stable)
    sample: np.ndarray = rng.standard_normal(3)
    return L @ sample


def _angle_deg(unit_a: np.ndarray, unit_b: np.ndarray) -> float:
    """Return the angle between two unit vectors in degrees."""
    dot: float = float(np.clip(np.dot(unit_a, unit_b), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def test_bootstrap_learns_gravity_stationary_startup() -> None:
    """Verify gravity bootstraps within 2 seconds for a stationary startup."""
    test_dir: Path = Path(__file__).resolve().parent
    resources_dir: Path = test_dir.parent / "resources"
    imu_path: Path = resources_dir / "test_imu_mpu6050_calibration.yaml"
    mag_path: Path = resources_dir / "test_mag_mmc5983ma_calibration.yaml"

    imu_frame: str = "imu"
    mag_frame: str = "mag"
    (
        imu_prior,
        cov_omega_raw,
        cov_a_raw,
        cov_omega_corr,
        cov_a_corr,
    ) = _load_imu_calibration(imu_path, frame_id=imu_frame)
    mag_cov: np.ndarray = _load_mag_covariance(mag_path)

    params: MountingParams = MountingParams.defaults()
    bootstrap = replace(params.bootstrap, bootstrap_sec=2.0)
    steady = replace(params.steady, steady_sec=1.0)
    mag_params = replace(params.mag, use_driver_cov_as_prior=False)
    params = params.replace(bootstrap=bootstrap, steady=steady, mag=mag_params)

    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    rng: np.random.Generator = np.random.default_rng(7)
    dt_sec: float = 0.02
    sample_count: int = 100

    g_mps2: float = 9.80665
    # Apply +0.1 g bias so the raw magnitude is near -10.8 m/s^2 at rest
    g_bias_scale: float = 1.1
    g_mag_mps2: float = g_mps2 * g_bias_scale
    gravity_unit: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    accel_base: np.ndarray = -g_mag_mps2 * gravity_unit

    mag_unit: np.ndarray = np.array([0.4, 0.2, -0.1], dtype=np.float64)
    mag_unit /= float(np.linalg.norm(mag_unit))
    mag_magnitude_T: float = 50e-6
    mag_base: np.ndarray = mag_magnitude_T * mag_unit

    eps: float = 1e-18
    A_inv: np.ndarray = np.linalg.solve(imu_prior.A_a, np.eye(3, dtype=np.float64))

    for sample_idx in range(sample_count):
        t_sec: float = sample_idx * dt_sec
        t_ns: int = int(round(t_sec * 1e9))

        # Small slow motion that should still be treated as stationary
        omega_slow: np.ndarray = 0.0005 * np.array(
            [
                np.sin(2.0 * np.pi * 0.1 * t_sec),
                np.cos(2.0 * np.pi * 0.1 * t_sec),
                np.sin(2.0 * np.pi * 0.05 * t_sec),
            ],
            dtype=np.float64,
        )
        omega_base: np.ndarray = np.array([0.001, -0.0015, 0.0008], dtype=np.float64)
        omega_noise: np.ndarray = _cholesky_noise(cov_omega_corr, rng=rng, eps=eps)
        omega_corr: np.ndarray = omega_base + omega_slow + omega_noise
        omega_raw: np.ndarray = omega_corr + imu_prior.b_g_rads

        accel_slow: np.ndarray = 0.05 * np.array(
            [
                np.sin(2.0 * np.pi * 0.05 * t_sec),
                np.cos(2.0 * np.pi * 0.07 * t_sec),
                np.sin(2.0 * np.pi * 0.03 * t_sec),
            ],
            dtype=np.float64,
        )
        accel_noise: np.ndarray = _cholesky_noise(cov_a_corr, rng=rng, eps=eps)
        accel_corr: np.ndarray = accel_base + accel_slow + accel_noise
        accel_raw: np.ndarray = A_inv @ accel_corr + imu_prior.b_a_mps2

        # Draw correlated mag noise using the driver covariance each sample
        mag_noise: np.ndarray = _cholesky_noise(mag_cov, rng=rng, eps=eps)
        mag_raw: np.ndarray = mag_base + mag_noise

        mag_packet: MagPacket = MagPacket(
            t_meas_ns=t_ns,
            frame_id=mag_frame,
            m_raw_T=mag_raw,
            cov_m_raw_T2=mag_cov,
        )
        imu_packet: ImuPacket = ImuPacket(
            t_meas_ns=t_ns,
            frame_id=imu_frame,
            omega_raw_rads=omega_raw,
            cov_omega_raw=cov_omega_raw,
            a_raw_mps2=accel_raw,
            cov_a_raw=cov_a_raw,
            calibration=imu_prior,
        )

        pipeline.ingest_mag(mag_packet=mag_packet)
        pipeline.ingest_imu_pair(imu_packet=imu_packet)
        pipeline.step(t_now_ns=t_ns)

        if sample_idx == 0:
            assert pipeline.is_bootstrapping()

    final_time_ns: int = int(round(2.0 * 1e9))
    pipeline.step(t_now_ns=final_time_ns)

    assert not pipeline.is_bootstrapping()
    assert pipeline.is_initialized()
    assert pipeline.steady_window_is_steady()
    gravity_est: np.ndarray | None = pipeline.gravity_direction_W_unit()
    assert gravity_est is not None
    # Compare angle to expected direction for noise-robust convergence
    gravity_error_deg: float = _angle_deg(gravity_est, gravity_unit)
    assert gravity_error_deg < 5.0

    flags: FlagsYaml | None = pipeline.current_flags()
    assert flags is not None
    assert flags.anchored is True
    assert flags.mag_reference_invalid is False
    assert flags.mag_disturbance_detected is False
    assert flags.mag_dir_prior_from_driver_cov is False
