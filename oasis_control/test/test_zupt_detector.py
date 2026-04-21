################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the IMU-only stationary-twist detector."""

from __future__ import annotations

import math

from oasis_control.localization.zupt_detector import ZuptDecision
from oasis_control.localization.zupt_detector import ZuptDetector
from oasis_control.localization.zupt_detector import ZuptDetectorConfig


QUIET_GYRO_RADS: tuple[float, float, float] = (0.0, 0.0, 0.0)
LOUD_GYRO_RADS: tuple[float, float, float] = (0.2, 0.0, 0.0)
QUIET_ACCEL_MPS2: tuple[float, float, float] = (0.0, 0.0, 0.0)
LOUD_ACCEL_MPS2: tuple[float, float, float] = (0.3, 0.0, 0.0)


def _make_detector() -> ZuptDetector:
    return ZuptDetector(
        ZuptDetectorConfig(
            min_stationary_sec=0.18,
            min_moving_sec=0.01,
            stationary_linear_velocity_sigma_mps=0.06,
            stationary_angular_velocity_sigma_rads=0.05,
            moving_linear_variance_mps2=1.0e6,
            moving_angular_variance_rads2=5.0e5,
        )
    )


def _accepted(decision: ZuptDecision | None) -> ZuptDecision:
    assert decision is not None
    return decision


def test_enters_stationary_after_quiet_dwell() -> None:
    detector: ZuptDetector = _make_detector()

    first_result: ZuptDecision = _accepted(
        detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    second_result: ZuptDecision = _accepted(
        detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert first_result.stationary is False
    assert first_result.reason == "enter_candidate_started"
    assert second_result.stationary is True
    assert second_result.reason == "stationary_asserted"


def test_quiet_dwell_resets_when_sample_exceeds_enter_threshold() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    interrupted_result: ZuptDecision = _accepted(
        detector.update(0.10, (0.07, 0.0, 0.0), QUIET_ACCEL_MPS2)
    )
    final_result: ZuptDecision = _accepted(
        detector.update(0.28, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert interrupted_result.stationary is False
    assert interrupted_result.reason == "moving"
    assert final_result.stationary is False
    assert final_result.reason == "enter_candidate_started"


def test_moving_vibration_does_not_false_assert_stationary() -> None:
    detector: ZuptDetector = _make_detector()

    samples: list[
        tuple[float, tuple[float, float, float], tuple[float, float, float]]
    ] = [
        (0.00, (0.07, 0.0, 0.0), (0.20, 0.0, 0.0)),
        (0.05, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
        (0.10, (0.10, 0.0, 0.0), (0.24, 0.0, 0.0)),
        (0.15, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
        (0.20, (0.12, 0.0, 0.0), (0.22, 0.0, 0.0)),
        (0.25, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
        (0.30, (0.10, 0.0, 0.0), (0.25, 0.0, 0.0)),
        (0.35, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
    ]

    decisions: list[ZuptDecision] = []
    for timestamp_sec, gyro_rads, accel_mps2 in samples:
        decisions.append(
            _accepted(detector.update(timestamp_sec, gyro_rads, accel_mps2))
        )

    assert all(decision.stationary is False for decision in decisions)
    assert decisions[-1].reason in {"moving", "enter_candidate_started"}


def test_exits_stationary_with_short_moving_dwell() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    result: ZuptDecision = _accepted(
        detector.update(0.19, LOUD_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.reason == "stationary_cleared"
    assert result.linear_zupt_variance_mps2 == 1.0e6
    assert result.angular_zupt_variance_rads2 == 5.0e5


def test_exits_stationary_with_loud_accel() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    result: ZuptDecision = _accepted(
        detector.update(0.19, QUIET_GYRO_RADS, LOUD_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.reason == "stationary_cleared"


def test_exit_happens_faster_than_enter() -> None:
    detector: ZuptDetector = _make_detector()

    initial_result: ZuptDecision = _accepted(
        detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    pending_enter_result: ZuptDecision = _accepted(
        detector.update(0.01, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    stationary_result: ZuptDecision = _accepted(
        detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    cleared_result: ZuptDecision = _accepted(
        detector.update(0.19, LOUD_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert initial_result.reason == "enter_candidate_started"
    assert pending_enter_result.stationary is False
    assert pending_enter_result.reason == "enter_candidate_pending"
    assert stationary_result.stationary is True
    assert cleared_result.stationary is False
    assert detector.state.stationary is False
    assert detector.state.last_reason == "stationary_cleared"


def test_stationary_variances_are_fixed_isotropic_values() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    result: ZuptDecision = _accepted(
        detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is True
    assert result.linear_zupt_variance_mps2 == 0.06**2
    assert result.angular_zupt_variance_rads2 == 0.05**2


def test_moving_variances_use_explicit_moving_defaults() -> None:
    detector: ZuptDetector = _make_detector()

    result: ZuptDecision = _accepted(
        detector.update(0.0, LOUD_GYRO_RADS, LOUD_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.linear_zupt_variance_mps2 == 1.0e6
    assert result.angular_zupt_variance_rads2 == 5.0e5


def test_invalid_imu_sample_is_reported_without_state_change() -> None:
    detector: ZuptDetector = _make_detector()

    initial_result: ZuptDecision = _accepted(
        detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    invalid_result: ZuptDecision = _accepted(
        detector.update(0.1, (math.nan, 0.0, 0.0), QUIET_ACCEL_MPS2)
    )

    assert initial_result.stationary is False
    assert invalid_result.stationary is False
    assert invalid_result.reason == "invalid_imu_sample"
    assert detector.state.last_timestamp_sec == 0.1


def test_invalid_timestamp_is_rejected() -> None:
    detector: ZuptDetector = _make_detector()

    assert detector.update(math.nan, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2) is None
    assert detector.state.last_reason == "invalid_timestamp"


def test_non_monotonic_timestamp_is_rejected_and_clears_candidates() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    assert detector.state.enter_candidate_start_sec == 0.0

    assert detector.update(-0.1, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2) is None
    assert detector.state.last_reason == "non_monotonic_timestamp"
    assert detector.state.enter_candidate_start_sec is None
    assert detector.state.exit_candidate_start_sec is None
