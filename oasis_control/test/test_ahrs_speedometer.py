################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-independent AHRS forward-speed estimator."""

from __future__ import annotations

import numpy as np
import pytest

from oasis_control.localization.ahrs_speedometer import AhrsImuSample
from oasis_control.localization.ahrs_speedometer import AhrsSpeedometer
from oasis_control.localization.ahrs_speedometer import AhrsSpeedometerConfig
from oasis_control.localization.ahrs_speedometer import FloatArray
from oasis_control.localization.ahrs_speedometer import SpeedometerEstimate
from oasis_control.localization.ahrs_speedometer import StationaryTwistObservation
from oasis_control.localization.ahrs_speedometer import _symmetrize
from oasis_control.localization.ahrs_speedometer import _valid_state_and_covariance


def test_requires_active_fresh_zupt_to_initialize() -> None:
    speedometer: AhrsSpeedometer = AhrsSpeedometer(AhrsSpeedometerConfig())

    estimate: SpeedometerEstimate | None = speedometer.update(make_imu(1.0))

    assert estimate is not None
    assert estimate.initialized is False
    assert estimate.zupt_applied is False


def test_integrates_signed_forward_acceleration_and_variance() -> None:
    speedometer: AhrsSpeedometer = initialized_speedometer()

    estimate: SpeedometerEstimate | None = speedometer.update(
        make_imu(1.1, accel_x_mps2=-2.0, accel_variance_mps4=0.5)
    )

    assert estimate is not None
    np.testing.assert_allclose(estimate.mean[0], -0.2)
    expected_variance: float = float(estimate.covariance[0, 0])
    assert expected_variance > 0.0


@pytest.mark.parametrize(
    ("acceleration_mps2", "expected_speed_mps"),
    [(2.0, 0.2), (-2.0, -0.2)],
)
def test_constant_acceleration_preserves_speed_sign(
    acceleration_mps2: float, expected_speed_mps: float
) -> None:
    speedometer: AhrsSpeedometer = initialized_speedometer()

    estimate: SpeedometerEstimate = require_estimate(
        speedometer.update(make_imu(1.1, accel_x_mps2=acceleration_mps2))
    )

    np.testing.assert_allclose(estimate.mean[0], expected_speed_mps)


def test_variance_propagation_uses_fallback_and_process_noise() -> None:
    config: AhrsSpeedometerConfig = AhrsSpeedometerConfig(
        fallback_accel_variance_mps4=4.0,
        speed_process_noise_mps2_per_sec=0.2,
    )
    speedometer: AhrsSpeedometer = initialized_speedometer(config)
    before: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(1.1)))

    after: SpeedometerEstimate = require_estimate(
        speedometer.update(make_imu(1.2, accel_variance_mps4=None))
    )

    assert after.used_accel_covariance_fallback is True
    np.testing.assert_allclose(
        after.covariance[0, 0],
        before.covariance[0, 0] + 0.1**2 * 4.0 + 0.2 * 0.1,
    )


def test_estimate_reports_angular_covariance_fallback() -> None:
    speedometer: AhrsSpeedometer = initialized_speedometer()
    unavailable: AhrsImuSample = make_imu(1.1)
    unavailable = AhrsImuSample(
        timestamp_ns=unavailable.timestamp_ns,
        accel_x_mps2=unavailable.accel_x_mps2,
        angular_velocity_rads=unavailable.angular_velocity_rads,
        accel_x_variance_mps4=unavailable.accel_x_variance_mps4,
        angular_covariance_rads2=None,
    )

    fallback: SpeedometerEstimate = require_estimate(speedometer.update(unavailable))
    measured: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(1.2)))

    assert fallback.used_angular_covariance_fallback is True
    assert measured.used_angular_covariance_fallback is False


def test_rejects_out_of_order_and_excessive_gap_without_integration() -> None:
    speedometer: AhrsSpeedometer = initialized_speedometer()

    assert speedometer.update(make_imu(1.1)) is not None
    assert speedometer.update(make_imu(1.1)) is None
    assert speedometer.last_rejection == "out_of_order"
    assert speedometer.update(make_imu(2.0, accel_x_mps2=10.0)) is None
    assert speedometer.last_rejection == "excessive_gap"
    estimate: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(2.1)))
    np.testing.assert_allclose(estimate.mean[0], 0.0, atol=1e-12)


def test_full_zupt_update_retains_cross_covariance_and_is_not_reused() -> None:
    config: AhrsSpeedometerConfig = AhrsSpeedometerConfig(zupt_freshness_sec=0.2)
    speedometer: AhrsSpeedometer = AhrsSpeedometer(config)
    covariance: FloatArray = np.multiply(np.eye(6, dtype=np.float64), 0.1)
    np.put(covariance, [3, 18], 0.02)
    speedometer.store_zupt(
        StationaryTwistObservation(
            timestamp_ns=1_000_000_000,
            mean=np.zeros(6, dtype=np.float64),
            covariance=covariance,
        )
    )
    speedometer.set_zupt_active(True)

    first: SpeedometerEstimate = require_estimate(
        speedometer.update(make_imu(1.0, angular=(1.0, 2.0, 3.0)))
    )
    second: SpeedometerEstimate = require_estimate(
        speedometer.update(make_imu(1.1, angular=(1.0, 2.0, 3.0)))
    )

    assert first.zupt_applied is True
    assert first.mean[3] != 1.0
    assert first.covariance[0, 3] != 0.0
    np.testing.assert_allclose(first.covariance, first.covariance.T)
    assert second.zupt_applied is False
    assert speedometer.last_zupt_status == "duplicate"
    np.testing.assert_allclose(second.covariance[0, 3:6], 0.0, atol=0.0)
    np.testing.assert_allclose(second.covariance[3:6, 0], 0.0, atol=0.0)
    assert np.min(np.linalg.eigvalsh(second.covariance)) >= -1.0e-10


def test_stale_and_singular_zupt_are_rejected_cleanly() -> None:
    speedometer: AhrsSpeedometer = AhrsSpeedometer(
        AhrsSpeedometerConfig(zupt_freshness_sec=0.01)
    )
    speedometer.set_zupt_active(True)
    speedometer.store_zupt(make_zupt(0.0, np.eye(6, dtype=np.float64)))
    stale: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(1.0)))
    assert stale.zupt_applied is False
    assert speedometer.last_zupt_status == "stale"

    singular: AhrsSpeedometer = AhrsSpeedometer(AhrsSpeedometerConfig())
    singular.set_zupt_active(True)
    singular.store_zupt(make_zupt(1.0, np.zeros((6, 6), dtype=np.float64)))
    result: SpeedometerEstimate = require_estimate(singular.update(make_imu(1.0)))
    assert result.zupt_applied is False
    assert singular.last_zupt_status == "singular_innovation"
    np.testing.assert_allclose(result.mean, np.zeros(6))
    assert np.all(np.isfinite(result.covariance))


def test_duplicate_and_out_of_order_zupt_are_not_stored() -> None:
    speedometer: AhrsSpeedometer = AhrsSpeedometer(AhrsSpeedometerConfig())
    first: StationaryTwistObservation = make_zupt(2.0, np.eye(6, dtype=np.float64))

    assert speedometer.store_zupt(first) is True
    assert speedometer.store_zupt(first) is False
    assert speedometer.last_zupt_status == "duplicate"
    assert speedometer.store_zupt(make_zupt(1.0, np.eye(6, dtype=np.float64))) is False
    assert speedometer.last_zupt_status == "out_of_order"


def test_integer_nanosecond_timestamp_ordering_is_exact() -> None:
    speedometer: AhrsSpeedometer = initialized_speedometer()

    assert speedometer.update(make_imu_ns(1_000_000_001)) is not None
    assert speedometer.update(make_imu_ns(1_000_000_001)) is None
    assert speedometer.last_rejection == "out_of_order"
    assert speedometer.update(make_imu_ns(1_000_000_000)) is None
    assert speedometer.last_rejection == "out_of_order"
    assert speedometer.update(make_imu_ns(1_000_000_002)) is not None


def test_zupt_freshness_nanosecond_boundary_is_inclusive() -> None:
    config: AhrsSpeedometerConfig = AhrsSpeedometerConfig(zupt_freshness_sec=10.0e-9)
    at_boundary: AhrsSpeedometer = AhrsSpeedometer(config)
    at_boundary.set_zupt_active(True)
    at_boundary.store_zupt(make_zupt_ns(1_000, np.eye(6, dtype=np.float64)))
    accepted: SpeedometerEstimate = require_estimate(
        at_boundary.update(make_imu_ns(1_010))
    )
    assert accepted.zupt_applied is True

    beyond: AhrsSpeedometer = AhrsSpeedometer(config)
    beyond.set_zupt_active(True)
    beyond.store_zupt(make_zupt_ns(1_000, np.eye(6, dtype=np.float64)))
    rejected: SpeedometerEstimate = require_estimate(beyond.update(make_imu_ns(1_011)))
    assert rejected.zupt_applied is False
    assert beyond.last_zupt_status == "stale"


def test_low_variance_zupt_corrects_speed_more_strongly() -> None:
    low_variance_speed: float = speed_after_second_zupt(0.001)
    high_variance_speed: float = speed_after_second_zupt(100.0)

    assert abs(low_variance_speed) < abs(high_variance_speed)


def test_complete_zupt_covariance_changes_coupled_correction() -> None:
    diagonal: FloatArray = np.multiply(np.eye(6, dtype=np.float64), 0.1)
    correlated: FloatArray = diagonal.copy()
    np.put(correlated, [3, 18], 0.08)

    diagonal_result: SpeedometerEstimate = apply_initial_zupt(diagonal)
    correlated_result: SpeedometerEstimate = apply_initial_zupt(correlated)

    assert correlated_result.mean[0] != diagonal_result.mean[0]
    assert correlated_result.covariance[0, 3] != 0.0


def test_covariance_validation_accepts_cleanup_and_rejects_corruption() -> None:
    state: np.ndarray = np.zeros(4, dtype=np.float64)
    asymmetric: np.ndarray = np.eye(4, dtype=np.float64)
    asymmetric[0, 1] += 1.0e-12

    assert _valid_state_and_covariance(state, _symmetrize(asymmetric)) is True
    materially_negative: np.ndarray = np.eye(4, dtype=np.float64)
    materially_negative[0, 0] = -1.0e-3
    assert _valid_state_and_covariance(state, materially_negative) is False
    nonfinite: np.ndarray = np.eye(4, dtype=np.float64)
    nonfinite[0, 0] = np.inf
    assert _valid_state_and_covariance(state, nonfinite) is False


def test_invalid_zupt_posterior_preserves_propagated_prior(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    speedometer: AhrsSpeedometer = AhrsSpeedometer(AhrsSpeedometerConfig())
    speedometer.set_zupt_active(True)
    speedometer.store_zupt(make_zupt_ns(1_000_000_000, np.eye(6, dtype=np.float64)))
    original_solve: object = np.linalg.solve

    def nonfinite_solve(left: np.ndarray, right: np.ndarray) -> np.ndarray:
        del left
        return np.full_like(right, np.nan)

    monkeypatch.setattr(np.linalg, "solve", nonfinite_solve)
    result: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(1.0)))
    monkeypatch.setattr(np.linalg, "solve", original_solve)

    assert result.zupt_applied is False
    assert speedometer.last_zupt_status == "invalid_posterior"
    np.testing.assert_allclose(result.mean, np.zeros(6))
    assert np.all(np.isfinite(result.covariance))


def initialized_speedometer(
    config: AhrsSpeedometerConfig | None = None,
) -> AhrsSpeedometer:
    speedometer: AhrsSpeedometer = AhrsSpeedometer(
        AhrsSpeedometerConfig() if config is None else config
    )
    covariance: FloatArray = np.multiply(np.eye(6, dtype=np.float64), 0.01)
    speedometer.store_zupt(make_zupt(1.0, covariance))
    speedometer.set_zupt_active(True)
    estimate: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(1.0)))
    assert estimate.initialized is True
    return speedometer


def make_imu(
    timestamp_sec: float,
    accel_x_mps2: float = 0.0,
    accel_variance_mps4: float | None = 0.2,
    angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> AhrsImuSample:
    return AhrsImuSample(
        timestamp_ns=round(timestamp_sec * 1_000_000_000),
        accel_x_mps2=accel_x_mps2,
        angular_velocity_rads=np.asarray(angular, dtype=np.float64),
        accel_x_variance_mps4=accel_variance_mps4,
        angular_covariance_rads2=np.array(
            [[0.2, 0.01, 0.02], [0.01, 0.3, 0.03], [0.02, 0.03, 0.4]],
            dtype=np.float64,
        ),
    )


def make_imu_ns(timestamp_ns: int) -> AhrsImuSample:
    sample: AhrsImuSample = make_imu(0.0)
    return AhrsImuSample(
        timestamp_ns=timestamp_ns,
        accel_x_mps2=sample.accel_x_mps2,
        angular_velocity_rads=sample.angular_velocity_rads,
        accel_x_variance_mps4=sample.accel_x_variance_mps4,
        angular_covariance_rads2=sample.angular_covariance_rads2,
    )


def make_zupt(
    timestamp_sec: float, covariance: FloatArray
) -> StationaryTwistObservation:
    return StationaryTwistObservation(
        timestamp_ns=round(timestamp_sec * 1_000_000_000),
        mean=np.zeros(6, dtype=np.float64),
        covariance=covariance,
    )


def make_zupt_ns(
    timestamp_ns: int, covariance: FloatArray
) -> StationaryTwistObservation:
    return StationaryTwistObservation(
        timestamp_ns=timestamp_ns,
        mean=np.zeros(6, dtype=np.float64),
        covariance=covariance,
    )


def speed_after_second_zupt(variance: float) -> float:
    speedometer: AhrsSpeedometer = initialized_speedometer()
    speedometer.update(make_imu(1.1, accel_x_mps2=10.0))
    covariance: FloatArray = np.multiply(np.eye(6, dtype=np.float64), variance)
    speedometer.store_zupt(make_zupt(1.2, covariance))
    estimate: SpeedometerEstimate = require_estimate(speedometer.update(make_imu(1.2)))
    return float(estimate.mean[0])


def apply_initial_zupt(covariance: FloatArray) -> SpeedometerEstimate:
    speedometer: AhrsSpeedometer = AhrsSpeedometer(AhrsSpeedometerConfig())
    speedometer.set_zupt_active(True)
    observation: StationaryTwistObservation = make_zupt(1.0, covariance)
    observation.mean[3] = 1.0
    speedometer.store_zupt(observation)
    return require_estimate(speedometer.update(make_imu(1.0, angular=(2.0, 0.0, 0.0))))


def require_estimate(estimate: SpeedometerEstimate | None) -> SpeedometerEstimate:
    assert estimate is not None
    return estimate
