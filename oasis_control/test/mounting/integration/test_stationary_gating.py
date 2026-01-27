################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Integration tests for steady gating in the mounting pipeline."""

from __future__ import annotations

from dataclasses import replace
from test.mounting.integration.sim_falcon import SAMPLE_DT_SEC
from test.mounting.integration.sim_falcon import FalconSample
from test.mounting.integration.sim_falcon import ImuCalibrationData
from test.mounting.integration.sim_falcon import MotionSegment
from test.mounting.integration.sim_falcon import load_imu_calibration
from test.mounting.integration.sim_falcon import load_mag_covariance
from test.mounting.integration.sim_falcon import pose_sequence
from test.mounting.integration.sim_falcon import resource_path
from test.mounting.integration.sim_falcon import simulate_falcon_sequence
from typing import Iterable
from typing import cast

import numpy as np

from oasis_control.localization.mounting.config.mounting_config import MountingConfig
from oasis_control.localization.mounting.config.mounting_params import MountingParams
from oasis_control.localization.mounting.config.mounting_params import SteadyParams
from oasis_control.localization.mounting.mounting_types import Diagnostics
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    MountingPipeline,
)
from oasis_control.localization.mounting.pipeline.mounting_pipeline import (
    PipelineOutputs,
)


# s, steady window duration used for gating assertions
STEADY_SEC: float = 0.5


def _steady_params(base: MountingParams) -> SteadyParams:
    """Return steady parameters tuned for the Falcon simulation."""
    return replace(
        base.steady,
        steady_sec=STEADY_SEC,
        omega_mean_thresh=0.05,
        omega_cov_thresh=1e-3,
        a_cov_thresh=0.05,
        a_norm_min=8.0,
        a_norm_max=13.0,
    )


def _assert_window_state(
    steady_history: list[tuple[int, bool]],
    *,
    start_ns: int,
    end_ns: int,
    expected: bool,
) -> None:
    """Assert steady state matches the expected value over a time window."""
    window: list[bool] = [
        state for t_ns, state in steady_history if start_ns <= t_ns <= end_ns
    ]
    assert window
    assert all(state is expected for state in window)


def _transition_times(history: list[tuple[int, int]]) -> list[int]:
    """Return timestamps when a counter changes value."""
    if not history:
        return []
    transitions: list[int] = []
    last_value: int = history[0][1]
    t_ns: int
    value: int
    for t_ns, value in history[1:]:
        if value != last_value:
            transitions.append(t_ns)
            last_value = value
    return transitions


def _in_any_window(t_ns: int, windows: Iterable[tuple[int, int]]) -> bool:
    """Return True when the timestamp falls inside any time window."""
    window: tuple[int, int]
    for window in windows:
        start_ns: int = window[0]
        end_ns: int = window[1]
        if start_ns <= t_ns <= end_ns:
            return True
    return False


def test_stationary_and_motion_gating_sequence() -> None:
    """Verify steady gating toggles across hold and motion segments."""
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
    )
    pipeline: MountingPipeline = MountingPipeline(config=MountingConfig(params))

    gravity_units: list[np.ndarray] = [
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
        np.array([0.25, -0.1, 0.96], dtype=np.float64),
        np.array([-0.2, 0.18, 0.96], dtype=np.float64),
    ]
    segments: list[MotionSegment] = pose_sequence(
        gravity_units=gravity_units,
        hold_sec=2.0,
        move_sec=0.6,
    )
    samples: list[FalconSample] = simulate_falcon_sequence(
        segments=segments,
        imu_calib=imu_calib,
        mag_cov=mag_cov,
        imu_frame_id=imu_frame,
        mag_frame_id=mag_frame,
    )

    steady_history: list[tuple[int, bool]] = []
    segment_history: list[tuple[int, int]] = []
    keyframe_history: list[tuple[int, int]] = []

    sample: FalconSample
    for sample in samples:
        if sample.mag_packet is not None:
            pipeline.ingest_mag(mag_packet=sample.mag_packet)
        pipeline.ingest_imu_pair(imu_packet=sample.imu_packet)
        outputs: PipelineOutputs = pipeline.step(t_now_ns=sample.t_ns)
        diagnostics: Diagnostics = cast(Diagnostics, outputs.diagnostics)

        steady_history.append((sample.t_ns, pipeline.steady_window_is_steady()))
        segment_history.append((sample.t_ns, diagnostics.segment_count))
        keyframe_history.append((sample.t_ns, diagnostics.keyframe_count))

    steady_ns: int = int(round(STEADY_SEC * 1e9))
    sample_period_ns: int = int(round(SAMPLE_DT_SEC * 1e9))
    hold1_end_ns: int = int(round(2.0 * 1e9))
    move1_end_ns: int = int(round(2.6 * 1e9))
    hold2_end_ns: int = int(round(4.6 * 1e9))
    move2_end_ns: int = int(round(5.2 * 1e9))
    hold3_end_ns: int = int(round(7.2 * 1e9))

    _assert_window_state(
        steady_history,
        start_ns=hold1_end_ns - int(round(0.4 * 1e9)),
        end_ns=hold1_end_ns - sample_period_ns,
        expected=True,
    )
    _assert_window_state(
        steady_history,
        start_ns=hold1_end_ns + int(round(0.2 * 1e9)),
        end_ns=move1_end_ns,
        expected=False,
    )
    _assert_window_state(
        steady_history,
        start_ns=move1_end_ns + steady_ns + int(round(0.2 * 1e9)),
        end_ns=hold2_end_ns - sample_period_ns,
        expected=True,
    )
    _assert_window_state(
        steady_history,
        start_ns=hold2_end_ns + int(round(0.2 * 1e9)),
        end_ns=move2_end_ns,
        expected=False,
    )
    _assert_window_state(
        steady_history,
        start_ns=move2_end_ns + steady_ns + int(round(0.2 * 1e9)),
        end_ns=hold3_end_ns - sample_period_ns,
        expected=True,
    )

    segment_transitions: list[int] = _transition_times(segment_history)
    hold_windows: list[tuple[int, int]] = [
        (steady_ns, hold1_end_ns - sample_period_ns),
        (move1_end_ns + steady_ns, hold2_end_ns - sample_period_ns),
        (move2_end_ns + steady_ns, hold3_end_ns - sample_period_ns),
    ]

    transition_time: int
    for transition_time in segment_transitions:
        assert _in_any_window(transition_time, hold_windows)

    final_keyframes: int = keyframe_history[-1][1]
    assert final_keyframes >= 2
