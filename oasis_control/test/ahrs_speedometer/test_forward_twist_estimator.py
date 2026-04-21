################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the yaw-only flat-surface forward-twist estimator."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.localization.speedometer.contracts import ZuptMeasurement
from oasis_control.localization.speedometer.forward_twist_estimator import (
    ForwardTwistEstimator,
)
from oasis_control.localization.speedometer.forward_twist_estimator import (
    make_flat_surface_twist_covariance,
)


IDENTITY_QUATERNION_XYZW: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
PITCH_UP_30_DEG_QUATERNION_XYZW: tuple[float, float, float, float] = (
    0.0,
    math.sin(math.radians(15.0)),
    0.0,
    math.cos(math.radians(15.0)),
)
ZERO_GYRO_RADS: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _make_estimator(
    *,
    persistence_path: str | None = None,
    yaw_learning_window_size: int = 6,
    yaw_learning_min_samples: int = 3,
    yaw_learning_accel_threshold_mps2: float = 0.1,
    yaw_learning_max_yaw_rate_rads: float = 0.2,
    yaw_learning_min_confidence: float = 0.8,
    yaw_learning_max_residual_ratio: float = 0.2,
    forward_accel_process_variance_mps2_2: float = 0.25,
) -> ForwardTwistEstimator:
    return ForwardTwistEstimator(
        config=ForwardTwistConfig(
            persistence_path=persistence_path,
            persistence_host="test-host",
            yaw_learning_window_size=yaw_learning_window_size,
            yaw_learning_min_samples=yaw_learning_min_samples,
            yaw_learning_accel_threshold_mps2=yaw_learning_accel_threshold_mps2,
            yaw_learning_max_yaw_rate_rads=yaw_learning_max_yaw_rate_rads,
            yaw_learning_min_confidence=yaw_learning_min_confidence,
            yaw_learning_max_residual_ratio=yaw_learning_max_residual_ratio,
            forward_accel_process_variance_mps2_2=(
                forward_accel_process_variance_mps2_2
            ),
        )
    )


def _feed_imu_samples(
    estimator: ForwardTwistEstimator,
    *,
    accel_samples_mps2: list[tuple[float, float, float]],
    yaw_rate_rads: float = 0.0,
    dt_ns: int = 100_000_000,
    start_timestamp_ns: int = 0,
) -> int:
    current_timestamp_ns: int = start_timestamp_ns
    for accel_sample_mps2 in accel_samples_mps2:
        estimator.update_imu(
            timestamp_ns=current_timestamp_ns,
            orientation_xyzw=IDENTITY_QUATERNION_XYZW,
            angular_velocity_rads=(0.0, 0.0, yaw_rate_rads),
            linear_acceleration_mps2=accel_sample_mps2,
        )
        current_timestamp_ns += dt_ns
    return current_timestamp_ns


def test_candidate_axis_treats_opposite_flat_excitation_as_compatible() -> None:
    estimator: ForwardTwistEstimator = _make_estimator(
        yaw_learning_window_size=4,
        yaw_learning_min_samples=5,
    )

    _feed_imu_samples(
        estimator,
        accel_samples_mps2=[
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (1.5, 0.0, 0.0),
            (-1.5, 0.0, 0.0),
        ],
    )
    estimate = estimator.get_estimate()

    assert estimate.candidate_forward_yaw_rad is not None
    assert math.isclose(estimate.candidate_forward_yaw_rad, 0.0, abs_tol=1.0e-6)
    assert estimate.candidate_confidence is not None
    assert estimate.candidate_confidence > 0.99
    assert estimate.checkpoint_commit_count == 0


def test_fresh_learning_commits_candidate_checkpoint_and_persists_metadata(
    tmp_path: Path,
) -> None:
    persistence_path: Path = tmp_path / "ahrs_speedometer_test-host.yaml"
    estimator: ForwardTwistEstimator = _make_estimator(
        persistence_path=str(persistence_path),
        yaw_learning_window_size=4,
        yaw_learning_min_samples=3,
    )

    _feed_imu_samples(
        estimator,
        accel_samples_mps2=[
            (0.0, 1.0, 0.0),
            (0.0, 1.2, 0.0),
            (0.0, 0.8, 0.0),
        ],
    )
    estimate = estimator.get_estimate()

    assert math.isclose(
        estimate.committed_forward_yaw_rad, math.pi / 2.0, abs_tol=1.0e-6
    )
    assert estimate.candidate_forward_yaw_rad is None
    assert estimate.checkpoint_commit_count == 1
    assert estimate.accepted_learning_sample_count == 3
    assert persistence_path.exists()
    persisted_yaml: str = persistence_path.read_text(encoding="utf-8")
    assert "checkpoint_count: 1" in persisted_yaml
    assert "fit_sample_count: 3" in persisted_yaml
    assert "confidence:" in persisted_yaml
    assert "residual:" in persisted_yaml
    assert "score:" in persisted_yaml
    assert "estimator: ahrs_speedometer" in persisted_yaml
    assert "last_update_reason: candidate_axis_checkpoint_commit" in persisted_yaml


def test_startup_load_freezes_learning_and_uses_persisted_yaw(tmp_path: Path) -> None:
    persistence_path: Path = tmp_path / "ahrs_speedometer_test-host.yaml"
    persistence_path.write_text(
        "\n".join(
            [
                "version: 1",
                "valid: true",
                "forward_yaw_rad: 1.1",
                "uncertainty_forward_yaw_rad: 0.2",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    estimator: ForwardTwistEstimator = _make_estimator(
        persistence_path=str(persistence_path),
    )
    original_yaml: str = persistence_path.read_text(encoding="utf-8")

    _feed_imu_samples(
        estimator,
        accel_samples_mps2=[
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        ],
        yaw_rate_rads=0.5,
    )
    estimate = estimator.get_estimate()

    assert estimate.loaded_from_persistence is True
    assert estimate.learning_enabled is False
    assert estimate.candidate_forward_yaw_rad is None
    assert estimate.checkpoint_commit_count == 0
    assert math.isclose(estimate.committed_forward_yaw_rad, 1.1, abs_tol=1.0e-9)
    assert persistence_path.read_text(encoding="utf-8") == original_yaml


def test_turn_detection_discards_uncommitted_candidate_but_keeps_committed_yaw() -> (
    None
):
    estimator: ForwardTwistEstimator = _make_estimator(
        yaw_learning_window_size=4,
        yaw_learning_min_samples=3,
    )

    next_timestamp_ns: int = _feed_imu_samples(
        estimator,
        accel_samples_mps2=[
            (1.0, 0.0, 0.0),
            (1.1, 0.0, 0.0),
            (0.9, 0.0, 0.0),
        ],
    )
    committed_estimate = estimator.get_estimate()

    next_timestamp_ns = _feed_imu_samples(
        estimator,
        accel_samples_mps2=[
            (0.0, 1.0, 0.0),
            (0.0, 1.1, 0.0),
        ],
        start_timestamp_ns=next_timestamp_ns,
    )
    candidate_estimate = estimator.get_estimate()
    assert candidate_estimate.candidate_forward_yaw_rad is not None

    estimator.update_imu(
        timestamp_ns=next_timestamp_ns,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=(0.0, 0.0, 0.5),
        linear_acceleration_mps2=(0.0, 1.0, 0.0),
    )
    after_turn_estimate = estimator.get_estimate()

    assert math.isclose(
        after_turn_estimate.committed_forward_yaw_rad,
        committed_estimate.committed_forward_yaw_rad,
        abs_tol=1.0e-9,
    )
    assert after_turn_estimate.candidate_forward_yaw_rad is None
    assert after_turn_estimate.turn_detected is True
    assert after_turn_estimate.checkpoint_discard_count == 1
    assert after_turn_estimate.discarded_uncommitted_sample_count == 3


def test_speed_propagation_uses_world_flat_projection_before_body_projection() -> None:
    estimator: ForwardTwistEstimator = _make_estimator(yaw_learning_min_samples=1)

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=PITCH_UP_30_DEG_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(1.0, 0.0, 0.0),
    )
    estimate = estimator.update_imu(
        timestamp_ns=1_000_000_000,
        orientation_xyzw=PITCH_UP_30_DEG_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(1.0, 0.0, 0.0),
    )

    assert estimate.forward_speed_mps > 0.70
    assert estimate.forward_speed_mps < 0.80
    assert estimate.forward_speed_variance_mps2 > 1.0


def test_speed_propagation_subtracts_learned_forward_accel_bias() -> None:
    estimator: ForwardTwistEstimator = _make_estimator(yaw_learning_min_samples=1)

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.5, 0.0, 0.0),
    )
    estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=100_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
        )
    )

    biased_estimate = estimator.update_imu(
        timestamp_ns=1_100_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.5, 0.0, 0.0),
    )

    assert biased_estimate.forward_accel_bias_mps2 > 0.0
    assert biased_estimate.forward_speed_mps < 0.3


def test_stationary_zupt_learns_persistent_forward_accel_bias() -> None:
    estimator: ForwardTwistEstimator = _make_estimator(yaw_learning_min_samples=1)

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.4, 0.0, 0.0),
    )
    estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=50_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
        )
    )
    estimate = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=100_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
        )
    )

    assert estimate.forward_accel_bias_mps2 > 0.2
    assert estimate.forward_accel_bias_variance_mps2_2 < 1.0


def test_negative_process_noise_settings_do_not_break_variance_floors() -> None:
    estimator: ForwardTwistEstimator = ForwardTwistEstimator(
        config=ForwardTwistConfig(
            yaw_learning_min_samples=1,
            forward_accel_process_variance_mps2_2=-10.0,
            forward_accel_bias_process_variance_mps2_2_per_sec=-5.0,
        )
    )

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.0, 0.0, 0.0),
    )
    estimate = estimator.update_imu(
        timestamp_ns=1_000_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.0, 0.0, 0.0),
    )

    assert estimate.forward_speed_variance_mps2 >= (
        ForwardTwistConfig.min_forward_speed_variance_mps2
    )
    assert estimate.forward_accel_bias_variance_mps2_2 >= (
        ForwardTwistConfig.min_forward_accel_bias_variance_mps2_2
    )


def test_speed_and_bias_variance_floors_remain_distinct_in_propagation() -> None:
    estimator: ForwardTwistEstimator = ForwardTwistEstimator(
        config=ForwardTwistConfig(
            yaw_learning_min_samples=1,
            min_forward_speed_variance_mps2=0.2,
            min_forward_accel_bias_variance_mps2_2=0.7,
            forward_accel_process_variance_mps2_2=-10.0,
            forward_accel_bias_process_variance_mps2_2_per_sec=-5.0,
        )
    )

    estimator._var_forward_speed_mps2 = 0.05
    estimator._var_forward_accel_bias_mps2_2 = 0.1

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.0, 0.0, 0.0),
    )
    estimate = estimator.update_imu(
        timestamp_ns=1_000_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(0.0, 0.0, 0.0),
    )

    assert estimate.forward_speed_variance_mps2 == pytest.approx(0.2)
    assert estimate.forward_accel_bias_variance_mps2_2 == pytest.approx(0.7)


def test_zupt_reduces_scalar_speed_even_when_learning_is_frozen(tmp_path: Path) -> None:
    persistence_path: Path = tmp_path / "ahrs_speedometer_test-host.yaml"
    persistence_path.write_text(
        "\n".join(
            [
                "version: 1",
                "valid: true",
                "forward_yaw_rad: 0.0",
                "uncertainty_forward_yaw_rad: 0.1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    estimator: ForwardTwistEstimator = _make_estimator(
        persistence_path=str(persistence_path),
    )

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(1.0, 0.0, 0.0),
    )
    before_zupt = estimator.update_imu(
        timestamp_ns=1_000_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(1.0, 0.0, 0.0),
    )
    after_zupt = estimator.update_zupt(
        measurement=ZuptMeasurement(
            timestamp_ns=1_100_000_000,
            zero_velocity_variance_mps2=0.01,
            stationary_flag=True,
        )
    )

    assert before_zupt.learning_enabled is False
    assert after_zupt.forward_speed_mps < before_zupt.forward_speed_mps
    assert (
        after_zupt.forward_speed_variance_mps2 < before_zupt.forward_speed_variance_mps2
    )


def test_reverse_motion_remains_possible_with_forward_accel_bias_state() -> None:
    estimator: ForwardTwistEstimator = _make_estimator(yaw_learning_min_samples=1)

    estimator.update_imu(
        timestamp_ns=0,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(-1.0, 0.0, 0.0),
    )
    estimate = estimator.update_imu(
        timestamp_ns=1_000_000_000,
        orientation_xyzw=IDENTITY_QUATERNION_XYZW,
        angular_velocity_rads=ZERO_GYRO_RADS,
        linear_acceleration_mps2=(-1.0, 0.0, 0.0),
    )

    assert estimate.forward_speed_mps < -0.9
    assert math.isclose(estimate.forward_accel_bias_mps2, 0.0, abs_tol=1.0e-9)


def test_covariance_adds_speed_and_yaw_terms_without_fake_z_variance() -> None:
    covariance: list[float] = make_flat_surface_twist_covariance(
        forward_speed_mps=2.0,
        forward_yaw_rad=math.pi / 4.0,
        forward_speed_variance_mps2=0.25,
        forward_yaw_variance_rad2=0.04,
        yaw_rate_variance_rads2=0.09,
        min_forward_speed_variance_mps2=(
            ForwardTwistConfig.min_forward_speed_variance_mps2
        ),
        min_forward_yaw_variance_rad2=ForwardTwistConfig.min_forward_yaw_variance_rad2,
        min_yaw_rate_variance_rads2=ForwardTwistConfig.min_yaw_rate_variance_rads2,
    )

    assert math.isclose(covariance[0], 0.205, rel_tol=1.0e-9, abs_tol=1.0e-9)
    assert math.isclose(covariance[1], 0.045, rel_tol=1.0e-9, abs_tol=1.0e-9)
    assert math.isclose(covariance[6], 0.045, rel_tol=1.0e-9, abs_tol=1.0e-9)
    assert math.isclose(covariance[7], 0.205, rel_tol=1.0e-9, abs_tol=1.0e-9)
    assert covariance[2] == 0.0
    assert covariance[8] == 0.0
    assert covariance[14] == 0.0
    assert covariance[35] == 0.09
