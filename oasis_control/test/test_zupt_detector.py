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
            min_stationary_sec=0.2,
            min_moving_sec=0.05,
            zupt_velocity_sigma_mps=0.06,
            moving_zupt_variance_mps2=1.0e6,
            stationary_variance_inflation=4.0,
        )
    )


def _accepted(decision: ZuptDecision | None) -> ZuptDecision:
    assert decision is not None
    return decision


def test_enters_stationary_with_quiet_imu() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    result: ZuptDecision = _accepted(
        detector.update(0.2, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is True
    assert result.reason == "stationary_asserted"


def test_exits_stationary_with_loud_gyro() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.2, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    result: ZuptDecision = _accepted(
        detector.update(0.25, LOUD_GYRO_RADS, QUIET_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.reason == "stationary_cleared"


def test_exits_stationary_with_loud_accel() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.2, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))

    result: ZuptDecision = _accepted(
        detector.update(0.25, QUIET_GYRO_RADS, LOUD_ACCEL_MPS2)
    )

    assert result.stationary is False
    assert result.reason == "stationary_cleared"


def test_stationary_variance_inflates_near_exit_threshold() -> None:
    detector: ZuptDetector = _make_detector()

    _accepted(detector.update(0.0, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    _accepted(detector.update(0.2, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2))
    quiet_result: ZuptDecision = _accepted(
        detector.update(0.21, QUIET_GYRO_RADS, QUIET_ACCEL_MPS2)
    )
    inflated_result: ZuptDecision = _accepted(
        detector.update(0.22, (0.08, 0.0, 0.0), QUIET_ACCEL_MPS2)
    )

    assert quiet_result.stationary is True
    assert inflated_result.stationary is True
    assert inflated_result.zupt_variance_mps2 > quiet_result.zupt_variance_mps2


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
