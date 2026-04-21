################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import math
from typing import Optional

from oasis_control.localization.ahrs.processing.attitude_uncertainty import (
    AhrsOrientationUncertaintyEstimator,
)
from oasis_control.localization.ahrs.processing.attitude_uncertainty import (
    AttitudeUncertaintyEstimate,
)
from oasis_control.localization.common.measurements.tilt_covariance import (
    gravity_covariance_to_tilt_variance_rad2,
)


def test_roll_and_pitch_variance_comes_from_gravity_covariance() -> None:
    estimator: AhrsOrientationUncertaintyEstimator = (
        AhrsOrientationUncertaintyEstimator()
    )
    gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81)
    gravity_covariance_mps2_2: tuple[tuple[float, float, float], ...] = (
        (0.04, 0.0, 0.0),
        (0.0, 0.09, 0.0),
        (0.0, 0.0, 0.01),
    )

    estimate = estimator.update(
        timestamp_ns=1_000_000_000,
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        gravity_mps2=gravity_mps2,
        gravity_covariance_mps2_2=gravity_covariance_mps2_2,
    )

    expected_variance_rad2: float = gravity_covariance_to_tilt_variance_rad2(
        gravity_mps2=gravity_mps2,
        gravity_covariance_mps2_2=gravity_covariance_mps2_2,
    )

    assert estimate.orientation_covariance_rad2 is not None
    assert estimate.orientation_covariance_unknown is False
    assert math.isclose(
        estimate.orientation_covariance_rad2[0][0],
        expected_variance_rad2,
        abs_tol=1.0e-12,
    )
    assert math.isclose(
        estimate.orientation_covariance_rad2[1][1],
        expected_variance_rad2,
        abs_tol=1.0e-12,
    )


def test_constant_yaw_has_zero_recent_jitter_variance() -> None:
    estimator: AhrsOrientationUncertaintyEstimator = (
        AhrsOrientationUncertaintyEstimator()
    )
    estimate: Optional[AttitudeUncertaintyEstimate] = None

    for index in range(4):
        estimate = estimator.update(
            timestamp_ns=index * 500_000_000,
            orientation_xyzw=_quaternion_from_yaw_rad(math.radians(25.0)),
            gravity_mps2=(0.0, 0.0, -9.81),
            gravity_covariance_mps2_2=_gravity_covariance(),
        )

    assert estimate is not None
    assert estimate.orientation_covariance_rad2 is not None
    assert math.isclose(
        estimate.orientation_covariance_rad2[2][2],
        0.0,
        abs_tol=1.0e-12,
    )


def test_small_noisy_yaw_produces_nonzero_recent_jitter_variance() -> None:
    estimator: AhrsOrientationUncertaintyEstimator = (
        AhrsOrientationUncertaintyEstimator()
    )
    yaw_values_deg: tuple[float, ...] = (100.0, 100.2, 99.8, 100.1, 99.9)
    estimate: Optional[AttitudeUncertaintyEstimate] = None

    index: int
    yaw_deg: float
    for index, yaw_deg in enumerate(yaw_values_deg):
        estimate = estimator.update(
            timestamp_ns=index * 500_000_000,
            orientation_xyzw=_quaternion_from_yaw_rad(math.radians(yaw_deg)),
            gravity_mps2=(0.0, 0.0, -9.81),
            gravity_covariance_mps2_2=_gravity_covariance(),
        )

    assert estimate is not None
    assert estimate.orientation_covariance_rad2 is not None
    expected_yaw_stddev_rad: float = _population_stddev_rad(
        tuple(math.radians(value_deg) for value_deg in yaw_values_deg)
    )
    assert math.isclose(
        estimate.orientation_covariance_rad2[2][2],
        expected_yaw_stddev_rad * expected_yaw_stddev_rad,
        abs_tol=1.0e-12,
    )


def test_wrapped_yaw_samples_stay_small_across_pi_boundary() -> None:
    estimator: AhrsOrientationUncertaintyEstimator = (
        AhrsOrientationUncertaintyEstimator()
    )
    wrapped_yaw_deg: tuple[float, ...] = (179.8, -179.9, 179.9, -179.8)
    estimate: Optional[AttitudeUncertaintyEstimate] = None

    index: int
    yaw_deg: float
    for index, yaw_deg in enumerate(wrapped_yaw_deg):
        estimate = estimator.update(
            timestamp_ns=index * 500_000_000,
            orientation_xyzw=_quaternion_from_yaw_rad(math.radians(yaw_deg)),
            gravity_mps2=(0.0, 0.0, -9.81),
            gravity_covariance_mps2_2=_gravity_covariance(),
        )

    assert estimate is not None
    assert estimate.orientation_covariance_rad2 is not None
    assert estimate.orientation_covariance_rad2[2][2] < math.radians(0.25) ** 2


def _quaternion_from_yaw_rad(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw_rad: float = 0.5 * yaw_rad
    return (
        0.0,
        0.0,
        math.sin(half_yaw_rad),
        math.cos(half_yaw_rad),
    )


def _gravity_covariance() -> tuple[tuple[float, float, float], ...]:
    return (
        (0.04, 0.0, 0.0),
        (0.0, 0.04, 0.0),
        (0.0, 0.0, 0.04),
    )


def _population_stddev_rad(values_rad: tuple[float, ...]) -> float:
    mean_rad: float = sum(values_rad) / float(len(values_rad))
    variance_rad2: float = sum(
        (value_rad - mean_rad) * (value_rad - mean_rad) for value_rad in values_rad
    ) / float(len(values_rad))
    return math.sqrt(variance_rad2)
