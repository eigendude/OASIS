################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for gravity-direction residual math."""

from __future__ import annotations

import math

import pytest

from oasis_control.localization.common.measurements.gravity_direction import (
    compute_gravity_direction_residual,
)


def test_gravity_residual_is_zero_for_consistent_attitude() -> None:
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(0.0, 0.0, -9.81),
        measured_gravity_covariance_mps2_2=(
            (0.04, 0.0, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, 0.0, 0.04),
        ),
        mounted_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert math.isclose(residual.residual_norm, 0.0, abs_tol=1.0e-9)
    assert math.isclose(residual.mahalanobis_distance, 0.0, abs_tol=1.0e-9)


def test_gravity_residual_is_nonzero_for_inconsistent_direction() -> None:
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(0.0, 9.81, 0.0),
        measured_gravity_covariance_mps2_2=(
            (0.04, 0.0, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, 0.0, 0.04),
        ),
        mounted_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert residual.residual_norm > 1.0
    assert math.isfinite(residual.mahalanobis_distance)


def test_gravity_residual_returns_nan_mahalanobis_without_covariance() -> None:
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(0.1, 0.0, -9.81),
        measured_gravity_covariance_mps2_2=None,
        mounted_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert residual.residual_norm > 0.0
    assert math.isnan(residual.mahalanobis_distance)


def test_gravity_residual_handles_very_small_covariance() -> None:
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(0.05, 0.0, -9.81),
        measured_gravity_covariance_mps2_2=(
            (1.0e-6, 0.0, 0.0),
            (0.0, 1.0e-6, 0.0),
            (0.0, 0.0, 1.0e-6),
        ),
        mounted_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert math.isfinite(residual.mahalanobis_distance)
    assert residual.mahalanobis_distance > 1.0


def test_gravity_residual_uses_predicted_direction_from_rotated_attitude() -> None:
    quarter_turn_about_y_xyzw: tuple[float, float, float, float] = (
        0.0,
        math.sin(math.pi / 4.0),
        0.0,
        math.cos(math.pi / 4.0),
    )
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(-9.81, 0.0, 0.0),
        measured_gravity_covariance_mps2_2=(
            (0.04, 0.0, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, 0.0, 0.04),
        ),
        mounted_orientation_xyzw=quarter_turn_about_y_xyzw,
    )

    assert math.isclose(residual.residual_norm, 0.0, abs_tol=1.0e-6)
    assert residual.predicted_direction[0] < -0.99


def test_gravity_residual_raises_for_zero_vector() -> None:
    with pytest.raises(ValueError, match="gravity vector norm must be positive"):
        compute_gravity_direction_residual(
            measured_gravity_mps2=(0.0, 0.0, 0.0),
            measured_gravity_covariance_mps2_2=None,
            mounted_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
