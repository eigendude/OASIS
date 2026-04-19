################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-agnostic HUD speedometer core."""

from __future__ import annotations

import math

from oasis_control.localization.speedometer_core import SpeedometerConfig
from oasis_control.localization.speedometer_core import SpeedometerCore
from oasis_control.localization.speedometer_core import SpeedometerEstimate


REST_ACCEL_MPS2: tuple[float, float, float] = (0.0, 0.0, 0.0)
FORWARD_ACCEL_MPS2: tuple[float, float, float] = (1.2, 0.0, 0.0)
QUIET_GYRO_RADS: tuple[float, float, float] = (0.0, 0.0, 0.0)
EXPECTED_FORWARD_AXIS: tuple[float, float, float] = (1.0, 0.0, 0.0)
VALID_ACCEL_COVARIANCE_MPS2_2: tuple[tuple[float, float, float], ...] = (
    (0.04, 0.0, 0.0),
    (0.0, 0.04, 0.0),
    (0.0, 0.0, 0.04),
)
NEGATIVE_DIAGONAL_COVARIANCE_MPS2_2: tuple[tuple[float, float, float], ...] = (
    (-1.0, 0.0, 0.0),
    (0.0, 0.04, 0.0),
    (0.0, 0.0, 0.04),
)
NAN_COVARIANCE_MPS2_2: tuple[tuple[float, float, float], ...] = (
    (math.nan, 0.0, 0.0),
    (0.0, 0.04, 0.0),
    (0.0, 0.0, 0.04),
)


def _make_core(
    axis_learning_accel_threshold_mps2: float = 0.1,
    axis_learning_min_samples: int = 3,
    axis_learning_min_confidence: float = 0.75,
    unlocked_speed_std_mps: float = 8.0,
    max_predict_timestamp_jitter_sec: float = 0.003,
) -> SpeedometerCore:
    return SpeedometerCore(
        SpeedometerConfig(
            axis_learning_accel_threshold_mps2=(axis_learning_accel_threshold_mps2),
            axis_learning_max_gyro_threshold_rads=1.0,
            axis_learning_min_samples=axis_learning_min_samples,
            axis_learning_min_confidence=axis_learning_min_confidence,
            unlocked_speed_std_mps=unlocked_speed_std_mps,
            default_forward_accel_std_mps2=0.2,
            process_bias_walk_std_mps2=0.01,
            initial_speed_std_mps=0.5,
            initial_bias_std_mps2=0.2,
            max_predict_timestamp_jitter_sec=max_predict_timestamp_jitter_sec,
        )
    )


def _accepted(estimate: SpeedometerEstimate | None) -> SpeedometerEstimate:
    assert estimate is not None
    return estimate


def _dot(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> float:
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2]


def _lock_axis(core: SpeedometerCore) -> SpeedometerEstimate:
    _accepted(core.handle_imu_sample(0.0, REST_ACCEL_MPS2, QUIET_GYRO_RADS))
    _accepted(
        core.handle_imu_sample(
            0.1,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
            VALID_ACCEL_COVARIANCE_MPS2_2,
        )
    )
    _accepted(
        core.handle_imu_sample(
            0.2,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
            VALID_ACCEL_COVARIANCE_MPS2_2,
        )
    )
    return _accepted(
        core.handle_imu_sample(
            0.3,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
            VALID_ACCEL_COVARIANCE_MPS2_2,
        )
    )


def test_initial_estimate_before_axis_lock() -> None:
    unlocked_speed_std_mps: float = 9.0
    core: SpeedometerCore = _make_core(unlocked_speed_std_mps=unlocked_speed_std_mps)

    estimate: SpeedometerEstimate = core.get_estimate()

    assert estimate.axis_learned is False
    assert estimate.speed_mps == 0.0
    assert estimate.speed_variance_mps2 == unlocked_speed_std_mps**2


def test_first_imu_sample_initializes_timing_without_prediction() -> None:
    unlocked_speed_std_mps: float = 7.0
    core: SpeedometerCore = _make_core(unlocked_speed_std_mps=unlocked_speed_std_mps)

    estimate: SpeedometerEstimate = _accepted(
        core.handle_imu_sample(0.0, REST_ACCEL_MPS2, QUIET_GYRO_RADS)
    )

    assert estimate.axis_learned is False
    assert estimate.speed_mps == 0.0
    assert estimate.speed_variance_mps2 == unlocked_speed_std_mps**2
    assert core.state.last_predict_timestamp_sec == 0.0


def test_axis_does_not_lock_before_minimum_samples() -> None:
    core: SpeedometerCore = _make_core(
        axis_learning_accel_threshold_mps2=0.05,
        axis_learning_min_samples=5,
        axis_learning_min_confidence=0.7,
    )

    _accepted(core.handle_imu_sample(0.0, REST_ACCEL_MPS2, QUIET_GYRO_RADS))
    _accepted(core.handle_imu_sample(0.1, FORWARD_ACCEL_MPS2, QUIET_GYRO_RADS))
    estimate: SpeedometerEstimate = _accepted(
        core.handle_imu_sample(0.2, FORWARD_ACCEL_MPS2, QUIET_GYRO_RADS)
    )

    assert estimate.axis_learned is False
    assert estimate.motion_axis_body is None


def test_axis_locks_after_enough_aligned_body_samples() -> None:
    axis_learning_min_confidence: float = 0.75
    core: SpeedometerCore = _make_core(
        axis_learning_accel_threshold_mps2=0.05,
        axis_learning_min_samples=3,
        axis_learning_min_confidence=axis_learning_min_confidence,
    )

    estimate: SpeedometerEstimate = _lock_axis(core)
    motion_axis_body: tuple[float, float, float] | None = estimate.motion_axis_body

    assert estimate.axis_learned is True
    assert motion_axis_body is not None
    assert math.isclose(
        abs(_dot(motion_axis_body, EXPECTED_FORWARD_AXIS)),
        1.0,
        abs_tol=2.0e-2,
    )
    assert estimate.axis_confidence == core.state.axis_confidence
    assert estimate.axis_confidence >= axis_learning_min_confidence


def test_axis_learning_uses_mounted_body_acceleration_signal() -> None:
    core: SpeedometerCore = _make_core(
        axis_learning_accel_threshold_mps2=0.5,
        axis_learning_min_samples=3,
    )

    estimate: SpeedometerEstimate = _lock_axis(core)
    motion_axis_body: tuple[float, float, float] | None = estimate.motion_axis_body

    assert estimate.axis_learned is True
    assert motion_axis_body is not None
    assert math.isclose(abs(motion_axis_body[0]), 1.0, abs_tol=2.0e-2)
    assert math.isclose(abs(motion_axis_body[2]), 0.0, abs_tol=2.0e-2)


def test_predict_step_grows_speed_after_axis_lock() -> None:
    core: SpeedometerCore = _make_core()

    locked_estimate: SpeedometerEstimate = _lock_axis(core)
    estimate: SpeedometerEstimate = _accepted(
        core.handle_imu_sample(
            0.4,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
            VALID_ACCEL_COVARIANCE_MPS2_2,
        )
    )

    assert estimate.axis_learned is True
    assert estimate.speed_mps > locked_estimate.speed_mps
    assert estimate.speed_mps > 0.0
    assert math.isfinite(estimate.speed_variance_mps2)
    assert estimate.speed_variance_mps2 > 0.0


def test_zupt_correction_reduces_speed_magnitude() -> None:
    core: SpeedometerCore = _make_core()

    _lock_axis(core)
    before_correction: SpeedometerEstimate = _accepted(
        core.handle_imu_sample(
            0.4,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
            VALID_ACCEL_COVARIANCE_MPS2_2,
        )
    )
    after_correction: SpeedometerEstimate = _accepted(core.handle_zupt(0.45, 0.01))

    assert after_correction.speed_mps < before_correction.speed_mps
    assert math.isfinite(after_correction.speed_variance_mps2)
    assert after_correction.speed_variance_mps2 > 0.0


def test_invalid_imu_timestamp_is_rejected() -> None:
    core: SpeedometerCore = _make_core()

    estimate: SpeedometerEstimate | None = core.handle_imu_sample(
        math.nan,
        REST_ACCEL_MPS2,
        QUIET_GYRO_RADS,
    )

    assert estimate is None


def test_clearly_stale_imu_timestamp_is_rejected() -> None:
    core: SpeedometerCore = _make_core(max_predict_timestamp_jitter_sec=0.003)

    _accepted(core.handle_imu_sample(1.0, REST_ACCEL_MPS2, QUIET_GYRO_RADS))

    estimate: SpeedometerEstimate | None = core.handle_imu_sample(
        0.99,
        FORWARD_ACCEL_MPS2,
        QUIET_GYRO_RADS,
    )

    assert estimate is None
    assert core.state.last_predict_timestamp_sec == 1.0


def test_small_backward_timestamp_jitter_is_dropped_quietly() -> None:
    unlocked_speed_std_mps: float = 6.0
    core: SpeedometerCore = _make_core(
        unlocked_speed_std_mps=unlocked_speed_std_mps,
        max_predict_timestamp_jitter_sec=0.003,
    )

    _accepted(core.handle_imu_sample(1.0, REST_ACCEL_MPS2, QUIET_GYRO_RADS))

    estimate: SpeedometerEstimate = _accepted(
        core.handle_imu_sample(
            0.9985,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
        )
    )

    assert estimate.axis_learned is False
    assert estimate.speed_mps == 0.0
    assert estimate.speed_variance_mps2 == unlocked_speed_std_mps**2
    assert core.state.last_predict_timestamp_sec == 1.0
    assert core.state.axis_sample_count == 0


def test_invalid_covariance_falls_back_cleanly() -> None:
    bad_covariance_mps2_2: tuple[tuple[float, float, float], ...]

    for bad_covariance_mps2_2 in (
        NEGATIVE_DIAGONAL_COVARIANCE_MPS2_2,
        NAN_COVARIANCE_MPS2_2,
    ):
        core: SpeedometerCore = _make_core()
        _lock_axis(core)

        estimate: SpeedometerEstimate = _accepted(
            core.handle_imu_sample(
                0.4,
                FORWARD_ACCEL_MPS2,
                QUIET_GYRO_RADS,
                bad_covariance_mps2_2,
            )
        )

        assert math.isfinite(estimate.speed_mps)
        assert math.isfinite(estimate.speed_variance_mps2)
        assert estimate.speed_variance_mps2 > 0.0


def test_stationary_hint_blocks_axis_learning_until_motion_resumes() -> None:
    core: SpeedometerCore = _make_core()

    _accepted(core.handle_imu_sample(0.0, REST_ACCEL_MPS2, QUIET_GYRO_RADS))
    _accepted(core.handle_zupt(0.05, 0.01))
    estimate: SpeedometerEstimate = _accepted(
        core.handle_imu_sample(
            0.10,
            FORWARD_ACCEL_MPS2,
            QUIET_GYRO_RADS,
            VALID_ACCEL_COVARIANCE_MPS2_2,
        )
    )

    assert core.state.stationary_hint is True
    assert estimate.axis_learned is False
    assert core.state.axis_sample_count == 0
