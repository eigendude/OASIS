################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the mounting pipeline."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any
from typing import cast

import numpy as np
import pytest

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_params import BootstrapParams
from oasis_control.localization.mounting.config.mounting_params import DiversityParams
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import SaveParams
from oasis_control.localization.mounting.config.mounting_params import StabilityParams
from oasis_control.localization.mounting.config.mounting_params import SteadyParams
from oasis_control.localization.mounting.config.mounting_params import TfParams
from oasis_control.localization.mounting.mounting_types import ImuCalibrationPrior
from oasis_control.localization.mounting.mounting_types import ImuPacket
from oasis_control.localization.mounting.mounting_types import MagPacket
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipeline,
)
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipelineError,
)
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    PipelineOutputs,
)
from oasis_control.localization.mounting.tf.tf_publisher import PublishedTransform
from oasis_control.localization.mounting.timing.ring_buffer import TimestampRingBuffer


_DEF_COV: np.ndarray = np.eye(3, dtype=np.float64)


def _calibration_prior(frame_id: str, *, valid: bool = True) -> ImuCalibrationPrior:
    """Create a simple calibration prior for tests."""
    return ImuCalibrationPrior(
        valid=valid,
        frame_id=frame_id,
        b_a_mps2=np.zeros(3, dtype=np.float64),
        A_a=np.eye(3, dtype=np.float64),
        b_g_rads=np.zeros(3, dtype=np.float64),
        cov_a_params=None,
        cov_b_g=None,
    )


def _imu_packet(t_ns: int, *, frame_id: str = "imu") -> ImuPacket:
    """Create a stationary IMU packet for tests."""
    return ImuPacket(
        t_meas_ns=t_ns,
        frame_id=frame_id,
        omega_raw_rads=np.zeros(3, dtype=np.float64),
        cov_omega_raw=_DEF_COV,
        a_raw_mps2=np.array([0.0, 0.0, -9.81], dtype=np.float64),
        cov_a_raw=_DEF_COV,
        calibration=_calibration_prior(frame_id),
    )


def _mag_packet(t_ns: int, *, frame_id: str = "mag") -> MagPacket:
    """Create a magnetometer packet for tests."""
    return MagPacket(
        t_meas_ns=t_ns,
        frame_id=frame_id,
        m_raw_T=np.array([0.2, 0.0, 0.0], dtype=np.float64),
        cov_m_raw_T2=_DEF_COV,
    )


def _pipeline_params(
    *,
    bootstrap_sec: float,
    steady_sec: float,
    stable_window_sec: float,
    stable_rot_thresh_rad: float,
    save_path: Path | None,
    save_period_sec: float,
) -> MountingParams:
    """Create mounting parameters for pipeline tests."""
    base_params: MountingParams = MountingParams.defaults()
    bootstrap: BootstrapParams = replace(
        base_params.bootstrap,
        bootstrap_sec=bootstrap_sec,
    )
    steady: SteadyParams = replace(
        base_params.steady,
        steady_sec=steady_sec,
    )
    stability: StabilityParams = replace(
        base_params.stability,
        stable_window_sec=stable_window_sec,
        stable_rot_thresh_rad=stable_rot_thresh_rad,
    )
    save: SaveParams = replace(
        base_params.save,
        save_period_sec=save_period_sec,
        output_path=str(save_path) if save_path is not None else None,
    )
    diversity: DiversityParams = replace(
        base_params.diversity,
        N_min=None,
        tilt_min_deg=None,
        yaw_min_deg=None,
    )
    tf_params: TfParams = replace(
        base_params.tf,
        publish_dynamic=False,
        publish_static_when_stable=True,
        republish_static_on_save=True,
    )
    params: MountingParams = replace(
        base_params,
        bootstrap=bootstrap,
        steady=steady,
        stability=stability,
        save=save,
        diversity=diversity,
        tf=tf_params,
    )
    return params


def test_construct_and_reset() -> None:
    """Ensure the pipeline constructs and resets cleanly."""
    params: MountingParams = MountingParams.defaults()
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))
    assert pipeline.is_bootstrapping()
    assert not pipeline.is_initialized()
    assert pipeline.keyframe_count() == 0
    assert pipeline.last_save_time_ns() is None

    pipeline.reset()
    assert pipeline.is_bootstrapping()
    assert not pipeline.is_initialized()
    assert pipeline.keyframe_count() == 0
    assert pipeline.last_save_time_ns() is None


def test_step_timestamp_monotonicity() -> None:
    """Ensure step rejects decreasing timestamps."""
    params: MountingParams = MountingParams.defaults()
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))
    pipeline.step(t_now_ns=100)
    with pytest.raises(MountingPipelineError):
        pipeline.step(t_now_ns=50)


def test_bootstrap_behavior() -> None:
    """Verify bootstrap completes after the configured duration."""
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.2,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=None,
        save_period_sec=10.0,
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    pipeline.ingest_imu_pair(imu_packet=_imu_packet(0))
    pipeline.step(t_now_ns=0)
    assert pipeline.is_bootstrapping()
    assert not pipeline.is_initialized()

    pipeline.ingest_imu_pair(imu_packet=_imu_packet(int(0.1e9)))
    pipeline.step(t_now_ns=int(0.1e9))
    assert pipeline.is_bootstrapping()

    pipeline.ingest_imu_pair(imu_packet=_imu_packet(int(0.3e9)))
    pipeline.step(t_now_ns=int(0.3e9))
    assert not pipeline.is_bootstrapping()
    assert pipeline.is_initialized()


def test_stability_and_tf_integration(tmp_path: Path) -> None:
    """Ensure stability transitions publish static TF and save republish."""
    output_path: Path = tmp_path / "mount.yaml"
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.05,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=output_path,
        save_period_sec=0.05,
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    pipeline.ingest_mag(mag_packet=_mag_packet(0))

    times_ns: list[int] = [
        0,
        int(0.05e9),
        int(0.1e9),
        int(0.15e9),
        int(0.2e9),
        int(0.25e9),
    ]
    outputs: list[tuple[int, bool, bool]] = []

    for t_ns in times_ns:
        pipeline.ingest_imu_pair(imu_packet=_imu_packet(t_ns))
        result: PipelineOutputs = pipeline.step(t_now_ns=t_ns)
        has_static: bool = any(
            transform.is_static for transform in result.published_transforms
        )
        outputs.append((t_ns, has_static, result.did_save))

    static_times: list[int] = [t_ns for t_ns, is_static, _ in outputs if is_static]
    assert static_times

    first_static_time: int = static_times[0]
    republished_on_save: bool = any(
        t_ns > first_static_time and is_static and did_save
        for t_ns, is_static, did_save in outputs
    )
    assert republished_on_save


def test_gravity_only_tf_publishes_static() -> None:
    """Ensure TF publishes without magnetometer data."""
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.05,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=None,
        save_period_sec=10.0,
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    times_ns: list[int] = [
        0,
        int(0.05e9),
        int(0.1e9),
        int(0.15e9),
        int(0.2e9),
        int(0.25e9),
    ]
    published: list[PublishedTransform] = []
    for t_ns in times_ns:
        pipeline.ingest_imu_pair(imu_packet=_imu_packet(t_ns))
        result: PipelineOutputs = pipeline.step(t_now_ns=t_ns)
        published.extend(result.published_transforms)

    assert published
    for transform in published:
        assert transform.parent_frame
        assert transform.child_frame

    child_frames: set[str] = {transform.child_frame for transform in published}
    assert child_frames == {"imu"}


def test_ring_buffer_tuple_iteration(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ensure tuple-based ring buffer iteration is handled."""
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.05,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=None,
        save_period_sec=10.0,
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    original_iter: Callable[[TimestampRingBuffer[MagPacket]], list[Any]]
    original_iter = TimestampRingBuffer.iter_time_order

    def _iter_time_order_tuple(
        self: TimestampRingBuffer[MagPacket],
    ) -> list[tuple[int, MagPacket]]:
        items: list[Any] = original_iter(self)
        return [(cast(int, np.int64(item.t_ns)), item.value) for item in items]

    monkeypatch.setattr(TimestampRingBuffer, "iter_time_order", _iter_time_order_tuple)

    mag_t_ns: int = 0
    pipeline.ingest_mag(mag_packet=_mag_packet(mag_t_ns))

    times_ns: list[int] = [
        0,
        int(0.05e9),
        int(0.1e9),
        int(0.15e9),
        int(0.2e9),
    ]
    published: list[PublishedTransform] = []
    for t_ns in times_ns:
        pipeline.ingest_imu_pair(imu_packet=_imu_packet(t_ns))
        result: PipelineOutputs = pipeline.step(t_now_ns=t_ns)
        published.extend(result.published_transforms)

    assert published
    child_frames: set[str] = {transform.child_frame for transform in published}
    assert "mag" in child_frames


def test_mag_frame_late_arrival_publishes_static_once() -> None:
    """Ensure static mag TF publishes after the frame arrives."""
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.05,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=None,
        save_period_sec=10.0,
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    imu_times_ns: list[int] = [
        0,
        int(0.05e9),
        int(0.1e9),
        int(0.15e9),
        int(0.2e9),
    ]
    outputs_before_mag: list[PipelineOutputs] = []
    for t_ns in imu_times_ns:
        pipeline.ingest_imu_pair(imu_packet=_imu_packet(t_ns))
        outputs_before_mag.append(pipeline.step(t_now_ns=t_ns))

    imu_static_transforms: list[PublishedTransform] = [
        transform
        for output in outputs_before_mag
        for transform in output.published_transforms
        if transform.is_static and transform.child_frame == "imu"
    ]
    assert imu_static_transforms

    mag_start_ns: int = int(0.25e9)
    pipeline.ingest_mag(mag_packet=_mag_packet(mag_start_ns))

    mag_times_ns: list[int] = [
        mag_start_ns,
        int(0.3e9),
        int(0.35e9),
        int(0.4e9),
    ]
    outputs_after_mag: list[PipelineOutputs] = []
    for t_ns in mag_times_ns:
        pipeline.ingest_imu_pair(imu_packet=_imu_packet(t_ns))
        outputs_after_mag.append(pipeline.step(t_now_ns=t_ns))

    mag_static_transforms: list[PublishedTransform] = [
        transform
        for output in outputs_after_mag
        for transform in output.published_transforms
        if transform.is_static and transform.child_frame == "mag"
    ]
    assert len(mag_static_transforms) == 1


def test_empty_output_path_disables_save() -> None:
    """Ensure empty output paths disable saving."""
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.05,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=None,
        save_period_sec=0.01,
    )
    save: SaveParams = replace(params.save, output_path="")
    params = replace(params, save=save)
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    times_ns: list[int] = [
        0,
        int(0.1e9),
        int(0.2e9),
    ]
    outputs: list[PipelineOutputs] = []
    for t_ns in times_ns:
        pipeline.ingest_imu_pair(imu_packet=_imu_packet(t_ns))
        outputs.append(pipeline.step(t_now_ns=t_ns))

    assert outputs
    assert all(not output.did_save for output in outputs)
    assert all(output.save_path is None for output in outputs)


def test_save_policy_failure() -> None:
    """Ensure save failures are handled without raising."""
    params: MountingParams = _pipeline_params(
        bootstrap_sec=0.05,
        steady_sec=0.05,
        stable_window_sec=0.1,
        stable_rot_thresh_rad=1.0,
        save_path=Path("/proc/invalid.yaml"),
        save_period_sec=0.01,
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    pipeline.ingest_mag(mag_packet=_mag_packet(0))
    pipeline.ingest_imu_pair(imu_packet=_imu_packet(0))
    pipeline.step(t_now_ns=0)

    pipeline.ingest_imu_pair(imu_packet=_imu_packet(int(0.2e9)))
    result: PipelineOutputs = pipeline.step(t_now_ns=int(0.2e9))

    assert not result.did_save
