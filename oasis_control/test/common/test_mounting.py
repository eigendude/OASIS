################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for fixed-frame mounting helpers shared by AHRS/common code."""

from __future__ import annotations

import math

from oasis_control.localization.common.data.accel_sample import AccelSample
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.data.imu_sample import ImuSample
from oasis_control.localization.common.frames.mounting import apply_mounting_to_accel
from oasis_control.localization.common.frames.mounting import apply_mounting_to_gravity
from oasis_control.localization.common.frames.mounting import apply_mounting_to_imu
from oasis_control.localization.common.frames.mounting import make_mounting_transform


def test_make_mounting_transform_sets_frames_and_rotation_matrix() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=quarter_turn_about_z_xyzw,
    )

    assert mounting_transform.parent_frame_id == "base_link"
    assert mounting_transform.child_frame_id == "imu_link"
    assert math.isclose(mounting_transform.rotation_matrix[0][1], -1.0, abs_tol=1.0e-9)
    assert math.isclose(mounting_transform.rotation_matrix[1][0], 1.0, abs_tol=1.0e-9)


def test_apply_mounting_to_imu_rotates_values_and_propagates_frame_id() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=quarter_turn_about_z_xyzw,
    )
    imu_sample = ImuSample(
        timestamp_ns=123,
        frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_rad2=((2.0, 0.0, 0.0), (0.0, 3.0, 0.0), (0.0, 0.0, 4.0)),
        orientation_covariance_unknown=False,
        angular_velocity_rads=(1.0, 0.0, 0.0),
        angular_velocity_covariance_rads2=(
            (2.0, 0.0, 0.0),
            (0.0, 3.0, 0.0),
            (0.0, 0.0, 4.0),
        ),
        linear_acceleration_mps2=(1.0, 0.0, 0.0),
        linear_acceleration_covariance_mps2_2=(
            (2.0, 0.0, 0.0),
            (0.0, 3.0, 0.0),
            (0.0, 0.0, 4.0),
        ),
    )

    mounted_sample = apply_mounting_to_imu(imu_sample, mounting_transform)

    assert mounted_sample.frame_id == "base_link"
    assert math.isclose(
        mounted_sample.orientation_xyzw[2], quarter_turn_about_z_xyzw[2]
    )
    assert math.isclose(mounted_sample.angular_velocity_rads[0], 0.0, abs_tol=1.0e-9)
    assert math.isclose(mounted_sample.angular_velocity_rads[1], 1.0, abs_tol=1.0e-9)
    assert mounted_sample.orientation_covariance_rad2 is not None
    assert math.isclose(
        mounted_sample.orientation_covariance_rad2[0][0], 3.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.orientation_covariance_rad2[0][1], 0.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.orientation_covariance_rad2[1][0], 0.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.orientation_covariance_rad2[1][1], 2.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.orientation_covariance_rad2[2][2], 4.0, abs_tol=1.0e-9
    )


def test_apply_mounting_to_imu_preserves_unknown_orientation_covariance() -> None:
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    imu_sample = ImuSample(
        timestamp_ns=124,
        frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_rad2=None,
        orientation_covariance_unknown=True,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_rads2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_mps2_2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )

    mounted_sample = apply_mounting_to_imu(imu_sample, mounting_transform)

    assert mounted_sample.orientation_covariance_unknown is True
    assert mounted_sample.orientation_covariance_rad2 is None


def test_apply_mounting_to_gravity_rotates_values_and_covariance() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=quarter_turn_about_z_xyzw,
    )
    gravity_sample = GravitySample(
        timestamp_ns=456,
        frame_id="imu_link",
        gravity_mps2=(1.0, 0.0, 0.0),
        gravity_covariance_mps2_2=((2.0, 0.5, 0.0), (0.5, 3.0, 0.0), (0.0, 0.0, 4.0)),
    )

    mounted_sample = apply_mounting_to_gravity(gravity_sample, mounting_transform)

    assert mounted_sample.frame_id == "base_link"
    assert math.isclose(mounted_sample.gravity_mps2[0], 0.0, abs_tol=1.0e-9)
    assert math.isclose(mounted_sample.gravity_mps2[1], 1.0, abs_tol=1.0e-9)
    assert mounted_sample.gravity_covariance_mps2_2 is not None
    assert math.isclose(
        mounted_sample.gravity_covariance_mps2_2[0][0], 3.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.gravity_covariance_mps2_2[0][1], -0.5, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.gravity_covariance_mps2_2[1][0], -0.5, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.gravity_covariance_mps2_2[1][1], 2.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.gravity_covariance_mps2_2[2][2], 4.0, abs_tol=1.0e-9
    )


def test_apply_mounting_to_gravity_accepts_missing_covariance() -> None:
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    gravity_sample = GravitySample(
        timestamp_ns=457,
        frame_id="imu_link",
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_mps2_2=None,
    )

    mounted_sample = apply_mounting_to_gravity(gravity_sample, mounting_transform)

    assert mounted_sample.gravity_covariance_mps2_2 is None


def test_apply_mounting_to_accel_rotates_values_and_covariance() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=quarter_turn_about_z_xyzw,
    )
    accel_sample = AccelSample(
        timestamp_ns=789,
        frame_id="imu_link",
        accel_mps2=(1.0, 0.0, 0.0),
        accel_covariance_mps2_2=((2.0, 0.5, 0.0), (0.5, 3.0, 0.0), (0.0, 0.0, 4.0)),
    )

    mounted_sample = apply_mounting_to_accel(accel_sample, mounting_transform)

    assert mounted_sample.frame_id == "base_link"
    assert math.isclose(mounted_sample.accel_mps2[0], 0.0, abs_tol=1.0e-9)
    assert math.isclose(mounted_sample.accel_mps2[1], 1.0, abs_tol=1.0e-9)
    assert mounted_sample.accel_covariance_mps2_2 is not None
    assert math.isclose(
        mounted_sample.accel_covariance_mps2_2[0][0], 3.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.accel_covariance_mps2_2[0][1], -0.5, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.accel_covariance_mps2_2[1][0], -0.5, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.accel_covariance_mps2_2[1][1], 2.0, abs_tol=1.0e-9
    )
    assert math.isclose(
        mounted_sample.accel_covariance_mps2_2[2][2], 4.0, abs_tol=1.0e-9
    )
