################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for reusable IMU and gravity validation helpers."""

from __future__ import annotations

import math

from oasis_control.localization.common.validation.gravity_validation import (
    validate_gravity_sample,
)
from oasis_control.localization.common.validation.imu_validation import (
    validate_imu_sample,
)


IDENTITY_COVARIANCE: tuple[float, ...] = (
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
)
IDENTITY_6X6_COVARIANCE: tuple[float, ...] = (
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)


def test_imu_validation_rejects_wrong_frame() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=1,
        frame_id="camera_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_frame"


def test_imu_validation_rejects_non_finite_quaternion() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=2,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, math.nan, 1.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_orientation"


def test_imu_validation_rejects_bad_angular_velocity() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=3,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.0, math.nan, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_angular_velocity"


def test_imu_validation_rejects_bad_linear_acceleration() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=4,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, math.nan),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_linear_acceleration"


def test_imu_validation_rejects_bad_orientation_covariance() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=5,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=(1.0,) * 8,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_orientation_covariance"


def test_imu_validation_rejects_bad_angular_covariance() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=6,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=(1.0,) * 8,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_angular_covariance"


def test_imu_validation_rejects_bad_linear_covariance() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=7,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=(1.0,) * 8,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_linear_covariance"


def test_imu_validation_accepts_unknown_orientation_covariance() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=8,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_row_major=(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is True
    assert validation_result.sample is not None
    assert validation_result.sample.orientation_covariance_unknown is True
    assert validation_result.sample.orientation_covariance_rad2 is None


def test_imu_validation_accepts_and_normalizes_good_sample() -> None:
    validation_result = validate_imu_sample(
        timestamp_ns=9,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 2.0),
        orientation_covariance_row_major=IDENTITY_COVARIANCE,
        angular_velocity_rads=(0.1, 0.2, 0.3),
        angular_velocity_covariance_row_major=IDENTITY_COVARIANCE,
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_row_major=IDENTITY_COVARIANCE,
    )

    assert validation_result.accepted is True
    assert validation_result.sample is not None
    assert validation_result.sample.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_gravity_validation_rejects_non_finite_vector() -> None:
    validation_result = validate_gravity_sample(
        timestamp_ns=10,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        gravity_mps2=(0.0, math.nan, -9.81),
        gravity_covariance_row_major=IDENTITY_6X6_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_vector"


def test_gravity_validation_rejects_zero_vector() -> None:
    validation_result = validate_gravity_sample(
        timestamp_ns=11,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        gravity_mps2=(0.0, 0.0, 0.0),
        gravity_covariance_row_major=IDENTITY_6X6_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_vector"


def test_gravity_validation_rejects_bad_frame() -> None:
    validation_result = validate_gravity_sample(
        timestamp_ns=12,
        frame_id="camera_link",
        expected_frame_id="imu_link",
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_row_major=IDENTITY_6X6_COVARIANCE,
    )

    assert validation_result.accepted is False
    assert validation_result.rejection_reason == "bad_frame"


def test_gravity_validation_accepts_valid_covariance() -> None:
    validation_result = validate_gravity_sample(
        timestamp_ns=13,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_row_major=IDENTITY_6X6_COVARIANCE,
    )

    assert validation_result.accepted is True
    assert validation_result.sample is not None
    assert validation_result.sample.gravity_covariance_mps2_2 == (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def test_gravity_validation_accepts_unusable_covariance_as_none() -> None:
    validation_result = validate_gravity_sample(
        timestamp_ns=14,
        frame_id="imu_link",
        expected_frame_id="imu_link",
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_row_major=(math.nan,) * 36,
    )

    assert validation_result.accepted is True
    assert validation_result.sample is not None
    assert validation_result.sample.gravity_covariance_mps2_2 is None
