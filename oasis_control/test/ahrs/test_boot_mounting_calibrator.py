################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the gravity-based boot AHRS mounting solve."""

from __future__ import annotations

import math

from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    BootMountingCalibrator,
)
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.frames.mounting import apply_mounting_to_gravity


def test_boot_mounting_solve_levels_measured_imu_gravity() -> None:
    calibrator: BootMountingCalibrator = BootMountingCalibrator(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=0.0,
        min_sample_count=1,
    )

    solution = calibrator.add_gravity_sample(
        GravitySample(
            timestamp_ns=2_000_000_000,
            frame_id="imu_link",
            gravity_mps2=(
                9.81 * math.sin(math.radians(20.0)),
                -9.81 * math.sin(math.radians(30.0)) * math.cos(math.radians(20.0)),
                -9.81 * math.cos(math.radians(30.0)) * math.cos(math.radians(20.0)),
            ),
            gravity_covariance_mps2_2=None,
        )
    )

    assert solution is not None
    assert math.isclose(solution.roll_rad, math.radians(30.0), abs_tol=1.0e-6)
    assert math.isclose(solution.pitch_rad, math.radians(20.0), abs_tol=1.0e-6)
    assert math.isclose(solution.yaw_rad, 0.0, abs_tol=1.0e-9)

    mounted_gravity = apply_mounting_to_gravity(
        GravitySample(
            timestamp_ns=2_000_000_000,
            frame_id="imu_link",
            gravity_mps2=(
                9.81 * math.sin(math.radians(20.0)),
                -9.81 * math.sin(math.radians(30.0)) * math.cos(math.radians(20.0)),
                -9.81 * math.cos(math.radians(30.0)) * math.cos(math.radians(20.0)),
            ),
            gravity_covariance_mps2_2=None,
        ),
        solution.mounting_transform,
    )

    gravity_norm_mps2: float = math.sqrt(
        mounted_gravity.gravity_mps2[0] * mounted_gravity.gravity_mps2[0]
        + mounted_gravity.gravity_mps2[1] * mounted_gravity.gravity_mps2[1]
        + mounted_gravity.gravity_mps2[2] * mounted_gravity.gravity_mps2[2]
    )
    assert math.isclose(mounted_gravity.gravity_mps2[0], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_gravity.gravity_mps2[1], 0.0, abs_tol=1.0e-6)
    assert math.isclose(
        mounted_gravity.gravity_mps2[2], -gravity_norm_mps2, abs_tol=1.0e-6
    )


def test_boot_mounting_requires_stationary_accepted_samples() -> None:
    calibrator: BootMountingCalibrator = BootMountingCalibrator(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=0.0,
        stationary_angular_speed_threshold_rads=0.2,
        min_sample_count=1,
    )

    rejected_solution = calibrator.add_gravity_sample(
        GravitySample(
            timestamp_ns=1_000_000_000,
            frame_id="imu_link",
            gravity_mps2=(0.0, 0.0, -9.81),
            gravity_covariance_mps2_2=None,
        ),
        angular_velocity_rads=(0.0, 0.0, 0.25),
    )
    accepted_solution = calibrator.add_gravity_sample(
        GravitySample(
            timestamp_ns=2_000_000_000,
            frame_id="imu_link",
            gravity_mps2=(0.0, 0.0, -9.81),
            gravity_covariance_mps2_2=None,
        ),
        angular_velocity_rads=(0.0, 0.0, 0.05),
    )

    assert rejected_solution is None
    assert accepted_solution is not None
    assert accepted_solution.sample_count == 1
