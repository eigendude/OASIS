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
from test.mounting.integration.sim_falcon import FalconSample
from test.mounting.integration.sim_falcon import ImuCalibrationData
from test.mounting.integration.sim_falcon import MotionSegment
from test.mounting.integration.sim_falcon import load_imu_calibration
from test.mounting.integration.sim_falcon import load_mag_covariance
from test.mounting.integration.sim_falcon import pose_sequence
from test.mounting.integration.sim_falcon import resource_path
from test.mounting.integration.sim_falcon import simulate_falcon_sequence

import numpy as np

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import SteadyParams
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipeline,
)
from oasis_control.localization.mounting.storage.yaml_format import FlagsYaml


# s, initial transient duration excluded from convergence checks
TRANSIENT_SEC: float = 0.5


def _angle_deg(unit_a: np.ndarray, unit_b: np.ndarray) -> float:
    """Return the angle between two unit vectors in degrees."""
    dot: float = float(np.clip(np.dot(unit_a, unit_b), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def _unit_vector(vector: np.ndarray) -> np.ndarray:
    """Return a unit-length vector for averaging reference directions."""
    array: np.ndarray = np.asarray(vector, dtype=np.float64)
    norm: float = float(np.linalg.norm(array))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("vector norm must be positive")
    return array / norm


def _steady_params(base: MountingParams) -> SteadyParams:
    """Return steady parameters tuned for the Falcon simulation."""
    return replace(
        base.steady,
        steady_sec=1.0,
        omega_mean_thresh=0.05,
        omega_cov_thresh=1e-3,
        a_cov_thresh=0.05,
        a_norm_min=8.0,
        a_norm_max=13.0,
    )


def test_bootstrap_learns_gravity_stationary_startup() -> None:
    """Verify gravity bootstraps within 2 seconds for a stationary startup."""
    imu_frame: str = "imu"
    mag_frame: str = "mag"

    imu_calib: ImuCalibrationData = load_imu_calibration(
        resource_path("test_imu_mpu6050_calibration.yaml"),
        frame_id=imu_frame,
    )
    mag_cov: np.ndarray = load_mag_covariance(
        resource_path("test_mag_mmc5983ma_calibration.yaml")
    )

    params: MountingParams = MountingParams.defaults()
    params = params.replace(
        bootstrap=replace(params.bootstrap, bootstrap_sec=2.0),
        steady=_steady_params(params),
        mag=replace(params.mag, use_driver_cov_as_prior=False),
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    gravity_unit: np.ndarray = _unit_vector(np.array([0.0, 0.0, 1.0], dtype=np.float64))
    segments: list[MotionSegment] = pose_sequence(
        gravity_units=[gravity_unit],
        hold_sec=2.0,
        move_sec=0.5,
    )
    samples: list[FalconSample] = simulate_falcon_sequence(
        segments=segments,
        imu_calib=imu_calib,
        mag_cov=mag_cov,
        imu_frame_id=imu_frame,
        mag_frame_id=mag_frame,
    )

    steady_seen: bool = False
    transient_ns: int = int(round(TRANSIENT_SEC * 1e9))
    sample: FalconSample
    for sample in samples:
        if sample.mag_packet is not None:
            pipeline.ingest_mag(mag_packet=sample.mag_packet)
        pipeline.ingest_imu_pair(imu_packet=sample.imu_packet)
        pipeline.step(t_now_ns=sample.t_ns)

        if sample.t_ns >= transient_ns:
            steady_seen = steady_seen or pipeline.steady_window_is_steady()

    final_time_ns: int = int(round(2.0 * 1e9))
    pipeline.step(t_now_ns=final_time_ns)

    assert steady_seen
    assert not pipeline.is_bootstrapping()
    assert pipeline.is_initialized()
    assert pipeline.steady_window_is_steady()

    gravity_est: np.ndarray | None = pipeline.gravity_direction_W_unit()
    assert gravity_est is not None

    ref_samples: list[np.ndarray] = [
        sample.gravity_unit for sample in samples if sample.t_ns >= transient_ns
    ]
    ref_mean: np.ndarray = np.mean(np.stack(ref_samples, axis=0), axis=0)
    gravity_ref: np.ndarray = _unit_vector(ref_mean)

    gravity_error_deg: float = _angle_deg(gravity_est, gravity_ref)
    assert gravity_error_deg < 5.0

    flags: FlagsYaml | None = pipeline.current_flags()
    assert flags is not None
    assert flags.anchored is True
    assert flags.mag_reference_invalid is False
    assert flags.mag_disturbance_detected is False
    assert flags.mag_dir_prior_from_driver_cov is False
