################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the IMU-only ZUPT detector."""

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
            min_moving_sec=0.05,
            enter_smoothing_time_constant_sec=0.12,
            zupt_velocity_sigma_mps=0.06,
            moving_zupt_variance_mps2=1.0e6,
            stationary_variance_inflation=4.0,
        )
    )


def _accepted(decision: ZuptDecision | None) -> ZuptDecision:
    assert decision is not None
    return decision


def test_quiet_stop_asserts_earlier_with_filtered_enter_evidence() -> None:
    filtered_detector: ZuptDetector = _make_detector()
    raw_detector: ZuptDetector = ZuptDetector(
        ZuptDetectorConfig(
            min_stationary_sec=0.18,
            min_moving_sec=0.05,
            enter_smoothing_time_constant_sec=0.0,
            zupt_velocity_sigma_mps=0.06,
            moving_zupt_variance_mps2=1.0e6,
            stationary_variance_inflation=4.0,
        )
    )

    samples: list[
        tuple[float, tuple[float, float, float], tuple[float, float, float]]
    ] = [
        (0.00, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
        (0.06, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
        (0.12, (0.07, 0.0, 0.0), QUIET_ACCEL_MPS2),
        (0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2),
    ]

    filtered_result: ZuptDecision | None = None
    raw_result: ZuptDecision | None = None
    for timestamp_sec, gyro_rads, accel_mps2 in samples:
        filtered_result = filtered_detector.update(timestamp_sec, gyro_rads, accel_mps2)
        raw_result = raw_detector.update(timestamp_sec, gyro_rads, accel_mps2)

    filtered_decision: ZuptDecision = _accepted(filtered_result)
    raw_decision: ZuptDecision = _accepted(raw_result)

    assert filtered_decision.stationary is True
    assert filtered_decision.reason == "stationary_asserted_filtered"
    assert filtered_decision.enter_evidence_source == "filtered"
    assert filtered_decision.filtered_gyro_norm_rads < 0.06
    assert raw_decision.stationary is False
    assert raw_decision.reason == "enter_candidate_started"
    assert raw_decision.enter_evidence_source == "raw"


def test_enters_stationary_with_quiet_imu() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    result: ZuptDecision = _accepted(
        detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is True
    assert result.reason == "stationary_asserted_filtered"
    assert result.enter_evidence_source == "filtered"


def test_moving_vibration_does_not_false_assert_stationary() -> None:
    detector: ZuptDetector = _make_detector()

    samples: list[
        tuple[float, tuple[float, float, float], tuple[float, float, float]]
    ] = [
        (0.00, (0.11, 0.0, 0.0), (0.20, 0.0, 0.0)),
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
    assert decisions[-1].reason in {
        "moving",
        "enter_candidate_started",
        "enter_candidate_pending_filtered",
    }


def test_exits_stationary_with_loud_gyro() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    result: ZuptDecision = _accepted(
        detector.update(0.23, LOUD_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.reason == "stationary_cleared"


def test_exits_stationary_with_loud_accel() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    result: ZuptDecision = _accepted(
        detector.update(0.23, QUIET_GYRO_RADS, LOUD_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.reason == "stationary_cleared"


def test_stationary_variance_inflates_near_exit_threshold() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    quiet_result: ZuptDecision = _accepted(
        detector.update(0.19, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    inflated_result: ZuptDecision = _accepted(
        detector.update(0.20, (0.08, 0.0, 0.0), QUIET_ACCEL_MPS2)
    )

    assert quiet_result.stationary is True
    assert inflated_result.stationary is True
    assert inflated_result.zupt_variance_mps2 > quiet_result.zupt_variance_mps2


def test_filtered_norms_are_deterministic() -> None:
    detector: ZuptDetector = ZuptDetector(
        ZuptDetectorConfig(
            min_stationary_sec=1.0,
            enter_smoothing_time_constant_sec=0.1,
        )
    )

    first_result: ZuptDecision = _accepted(
        detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    second_result: ZuptDecision = _accepted(
        detector.update(0.1, (0.1, 0.0, 0.0), (0.2, 0.0, 0.0))
    )
    alpha: float = 1.0 - math.exp(-1.0)

    assert first_result.filtered_gyro_norm_rads == 0.0
    assert first_result.filtered_accel_norm_mps2 == 0.0
    assert second_result.enter_evidence_source == "filtered"
    assert math.isclose(second_result.filtered_gyro_norm_rads, alpha * 0.1)
    assert math.isclose(second_result.filtered_accel_norm_mps2, alpha * 0.2)


def test_raw_enter_evidence_is_reported_when_smoothing_disabled() -> None:
    detector: ZuptDetector = ZuptDetector(
        ZuptDetectorConfig(
            min_stationary_sec=0.18,
            enter_smoothing_time_constant_sec=0.0,
        )
    )

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    result: ZuptDecision = _accepted(
        detector.update(0.18, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is True
    assert result.reason == "stationary_asserted_raw"
    assert result.enter_evidence_source == "raw"
    assert result.filtered_gyro_norm_rads == result.gyro_norm_rads
    assert result.filtered_accel_norm_mps2 == result.accel_norm_mps2


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
    assert invalid_result.enter_evidence_source == "filtered"


def test_invalid_timestamp_is_rejected() -> None:
    detector: ZuptDetector = _make_detector()

    result: ZuptDecision | None = detector.update(
        math.nan, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2
    )

    assert result is None
    assert detector.state.last_reason == "invalid_timestamp"
    assert detector.state.last_timestamp_sec is None


def test_non_monotonic_timestamp_is_rejected_and_resets_candidates() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(1.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    assert detector.state.enter_candidate_start_sec == 1.0

    result: ZuptDecision | None = detector.update(
        0.9, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2
    )

    assert result is None
    assert detector.state.last_reason == "non_monotonic_timestamp"
    assert detector.state.enter_candidate_start_sec is None
    assert detector.state.exit_candidate_start_sec is None
    assert detector.state.last_timestamp_sec == 1.0
