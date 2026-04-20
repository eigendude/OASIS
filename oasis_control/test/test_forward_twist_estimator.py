################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-agnostic AHRS forward-twist estimator."""

from __future__ import annotations

import math
from pathlib import Path
from typing import cast

import yaml  # type: ignore[import-untyped]

from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.localization.speedometer.contracts import ForwardTwistEstimate
from oasis_control.localization.speedometer.contracts import ZuptMeasurement
from oasis_control.localization.speedometer.forward_twist_estimator import (
    ForwardTwistEstimator,
)
from oasis_control.localization.speedometer.forward_yaw_persistence import (
    ForwardYawPersistence,
)
from oasis_control.localization.speedometer.turn_detector import TurnDetector


IDENTITY_QUATERNION_XYZW: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
QUIET_GYRO_RADS: tuple[float, float, float] = (0.0, 0.0, 0.0)
FORWARD_ACCEL_MPS2: tuple[float, float, float] = (1.2, 0.0, 0.0)
TURN_ACCEL_MPS2: tuple[float, float, float] = (0.0, 1.2, 0.0)
REVERSE_ACCEL_MPS2: tuple[float, float, float] = (-1.2, 0.0, 0.0)


def _make_estimator(mount_info_directory: Path) -> ForwardTwistEstimator:
    config: ForwardTwistConfig = ForwardTwistConfig(
        learning_accel_threshold_mps2=0.1,
        learning_min_samples=4,
        learning_min_confidence=0.7,
        checkpoint_min_samples=3,
        checkpoint_max_candidate_delta_rad=0.8,
        initial_forward_speed_sigma_mps=0.5,
        min_forward_speed_variance_mps2=1.0e-6,
        forward_accel_process_sigma_mps2=0.2,
        max_imu_dt_sec=0.2,
        turn_rate_threshold_rads=0.4,
        turn_accel_threshold_mps2=0.1,
        turn_direction_alignment_threshold=0.75,
        persistence_write_on_checkpoint=True,
    )
    persistence: ForwardYawPersistence = ForwardYawPersistence(
        hostname="falcon",
        mount_info_directory=mount_info_directory,
    )
    turn_detector: TurnDetector = TurnDetector(
        turn_rate_threshold_rads=config.turn_rate_threshold_rads,
        turn_accel_threshold_mps2=config.turn_accel_threshold_mps2,
        turn_direction_alignment_threshold=(config.turn_direction_alignment_threshold),
    )
    return ForwardTwistEstimator(
        config=config,
        persistence=persistence,
        turn_detector=turn_detector,
    )


def _learn_committed_axis(estimator: ForwardTwistEstimator) -> ForwardTwistEstimate:
    estimator.update_imu(
        timestamp_ns=100_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=FORWARD_ACCEL_MPS2,
    )
    estimator.update_imu(
        timestamp_ns=200_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=FORWARD_ACCEL_MPS2,
    )
    estimator.update_imu(
        timestamp_ns=300_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=FORWARD_ACCEL_MPS2,
    )
    return estimator.update_imu(
        timestamp_ns=400_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=FORWARD_ACCEL_MPS2,
    )


def _build_negative_speed(estimator: ForwardTwistEstimator) -> ForwardTwistEstimate:
    estimator.update_imu(
        timestamp_ns=500_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=REVERSE_ACCEL_MPS2,
    )
    estimator.update_imu(
        timestamp_ns=600_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=REVERSE_ACCEL_MPS2,
    )
    return estimator.update_imu(
        timestamp_ns=700_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=REVERSE_ACCEL_MPS2,
    )


def test_first_motion_sign_locks_forward_axis_and_commits(
    tmp_path: Path,
) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    estimate: ForwardTwistEstimate = _learn_committed_axis(estimator)

    assert estimate.forward_axis.learned is True
    assert estimate.learning_state.forward_sign_locked is True
    assert estimate.learning_state.checkpoint_just_committed is True
    assert estimate.learning_state.checkpoint_commit_count == 1
    assert math.isclose(estimate.forward_axis.forward_yaw_rad, 0.0, abs_tol=1.0e-3)
    assert math.isclose(estimate.forward_axis.forward_axis_xyz[0], 1.0, abs_tol=1.0e-3)
    assert math.isclose(estimate.forward_axis.forward_axis_xyz[1], 0.0, abs_tol=1.0e-3)
    assert estimator.persistence_success_count == 1


def test_turn_discards_uncommitted_learning_and_keeps_committed_axis(
    tmp_path: Path,
) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)
    committed_estimate: ForwardTwistEstimate = _learn_committed_axis(estimator)

    estimator.update_imu(
        timestamp_ns=500_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=(1.0, 0.2, 0.0),
    )
    discarded_estimate: ForwardTwistEstimate = estimator.update_imu(
        timestamp_ns=600_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=(0.0, 0.0, 0.5),
        linear_acceleration_mps2=TURN_ACCEL_MPS2,
    )

    assert discarded_estimate.turn_detected is True
    assert discarded_estimate.learning_state.learning_gated_by_turn is True
    assert discarded_estimate.learning_state.uncommitted_sample_count == 0
    assert discarded_estimate.learning_state.checkpoint_discard_count == 1
    assert math.isclose(
        discarded_estimate.learning_state.committed_forward_yaw_rad,
        committed_estimate.learning_state.committed_forward_yaw_rad,
        abs_tol=1.0e-6,
    )


def test_zupt_reduces_speed_magnitude_and_variance(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    _learn_committed_axis(estimator)
    before_zupt: ForwardTwistEstimate = estimator.update_imu(
        timestamp_ns=500_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=(0.1, 0.0, 0.0),
    )
    after_zupt: ForwardTwistEstimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=450_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
            stationary_flag_timestamp_ns=450_000_000,
        )
    )

    assert before_zupt.forward_speed_mps > 0.0
    assert after_zupt.forward_speed_mps < before_zupt.forward_speed_mps
    assert (
        after_zupt.forward_speed_variance_mps2 < before_zupt.forward_speed_variance_mps2
    )
    assert math.isclose(
        after_zupt.forward_speed_sigma_mps,
        math.sqrt(after_zupt.forward_speed_variance_mps2),
        rel_tol=1.0e-6,
    )
    assert after_zupt.zupt_update_applied is True
    assert after_zupt.zupt_rejected_stale is False
    assert after_zupt.zupt_rejected_motion_contradiction is False
    assert after_zupt.zupt_measurement_variance_used_mps2 == 0.01
    assert after_zupt.zupt_kalman_gain > 0.0


def test_non_stationary_zupt_is_ignored(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    _learn_committed_axis(estimator)
    before_zupt: ForwardTwistEstimate = estimator.get_estimate()
    after_zupt: ForwardTwistEstimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=350_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=False,
            stationary_flag_timestamp_ns=350_000_000,
        )
    )

    assert after_zupt.zupt_sample_rejected is True
    assert after_zupt.forward_speed_mps == before_zupt.forward_speed_mps
    assert estimator.zupt_drop_count == 1


def test_stale_zupt_flag_does_not_zero_speed(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    _learn_committed_axis(estimator)
    before_zupt: ForwardTwistEstimate = estimator.update_imu(
        timestamp_ns=500_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=FORWARD_ACCEL_MPS2,
    )
    after_zupt: ForwardTwistEstimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=520_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
            stationary_flag_timestamp_ns=250_000_000,
        )
    )

    assert after_zupt.zupt_sample_rejected is True
    assert after_zupt.zupt_rejected_stale is True
    assert after_zupt.zupt_update_applied is False
    assert after_zupt.forward_speed_mps == before_zupt.forward_speed_mps
    assert after_zupt.zupt_rejected_stale_count == 1


def test_stale_zupt_does_not_zero_speed(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    _learn_committed_axis(estimator)
    before_zupt: ForwardTwistEstimate = estimator.update_imu(
        timestamp_ns=800_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=QUIET_GYRO_RADS,
        linear_acceleration_mps2=FORWARD_ACCEL_MPS2,
    )
    after_zupt: ForwardTwistEstimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=500_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
            stationary_flag_timestamp_ns=500_000_000,
        )
    )

    assert after_zupt.zupt_sample_rejected is True
    assert after_zupt.zupt_rejected_stale is True
    assert after_zupt.forward_speed_mps == before_zupt.forward_speed_mps


def test_contradictory_motion_rejects_bad_zupt(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    _learn_committed_axis(estimator)
    reverse_estimate: ForwardTwistEstimate = _build_negative_speed(estimator)
    rejected_estimate: ForwardTwistEstimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=710_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
            stationary_flag_timestamp_ns=710_000_000,
        )
    )

    assert reverse_estimate.forward_speed_mps < 0.0
    assert rejected_estimate.zupt_sample_rejected is True
    assert rejected_estimate.zupt_rejected_motion_contradiction is True
    assert rejected_estimate.zupt_update_applied is False
    assert rejected_estimate.forward_speed_mps == reverse_estimate.forward_speed_mps
    assert rejected_estimate.zupt_rejected_motion_count == 1


def test_reverse_motion_is_not_collapsed_by_mistimed_zupt(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)

    _learn_committed_axis(estimator)
    reverse_estimate: ForwardTwistEstimate = _build_negative_speed(estimator)
    after_stale_zupt: ForwardTwistEstimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=450_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
            stationary_flag_timestamp_ns=450_000_000,
        )
    )

    assert reverse_estimate.forward_speed_mps < -0.1
    assert after_stale_zupt.zupt_sample_rejected is True
    assert after_stale_zupt.zupt_rejected_stale is True
    assert after_stale_zupt.forward_speed_mps < -0.1
    assert math.isclose(
        after_stale_zupt.forward_speed_mps,
        reverse_estimate.forward_speed_mps,
        abs_tol=1.0e-9,
    )


def test_persistence_payload_matches_expected_shape(tmp_path: Path) -> None:
    estimator: ForwardTwistEstimator = _make_estimator(tmp_path)
    estimate: ForwardTwistEstimate = _learn_committed_axis(estimator)

    persistence_path: Path = tmp_path / "forward_twist_falcon.yaml"
    assert persistence_path.exists()

    payload: dict[str, object] = yaml.safe_load(
        persistence_path.read_text(encoding="utf-8")
    )
    forward_yaw_value: float | int | str = cast(
        float | int | str,
        payload["forward_yaw_rad"],
    )

    assert payload["version"] == 1
    assert payload["host"] == "falcon"
    assert payload["estimator"] == "ahrs_forward_twist"
    assert payload["valid"] is True
    assert math.isclose(
        float(forward_yaw_value),
        estimate.learning_state.committed_forward_yaw_rad,
        abs_tol=1.0e-6,
    )
    assert payload["checkpoint_count"] == 1
    assert payload["fit_sample_count"] == 4
    assert payload["forward_axis"] == [1.0, 0.0, 0.0]
