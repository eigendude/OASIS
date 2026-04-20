################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# mypy: disable-error-code=import-not-found

"""Tests for the ROS-facing AHRS speedometer stationary-twist contract."""

from __future__ import annotations

from typing import Any

import pytest
import rclpy
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg

from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.nodes.ahrs.ahrs_speedometer_node import AhrsSpeedometerNode
from oasis_control.nodes.ahrs.ahrs_speedometer_node import _default_persistence_path
from oasis_control.nodes.ahrs.ahrs_speedometer_node import (
    _extract_stationary_linear_variance_from_zupt_covariance,
)


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_extracts_scalar_variance_from_linear_covariance_block_mean() -> None:
    covariance: list[float] = [0.0] * 36
    covariance[0] = 0.2
    covariance[7] = 0.5
    covariance[14] = 0.8
    covariance[21] = 9.0
    covariance[28] = 10.0
    covariance[35] = 11.0

    variance_mps2: float = _extract_stationary_linear_variance_from_zupt_covariance(
        covariance
    )

    assert variance_mps2 == pytest.approx(0.5)


def test_extracts_isotropic_linear_variance_and_ignores_angular_block() -> None:
    covariance: list[float] = [0.0] * 36
    covariance[0] = 0.04
    covariance[7] = 0.04
    covariance[14] = 0.04
    covariance[21] = 100.0
    covariance[28] = 200.0
    covariance[35] = 300.0

    variance_mps2: float = _extract_stationary_linear_variance_from_zupt_covariance(
        covariance
    )

    assert variance_mps2 == pytest.approx(0.04)


def test_default_persistence_path_uses_ahrs_speedometer_filename(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "oasis_control.nodes.ahrs.ahrs_speedometer_node.socket.gethostname",
        lambda: "test-host",
    )

    assert _default_persistence_path().endswith("ahrs_speedometer_test-host.yaml")


def test_ahrs_speedometer_accepts_6d_stationary_twist_zupt() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0, linear_accel_x=1.0))
        node._handle_imu(make_imu_message(timestamp_sec=1.0, linear_accel_x=1.0))

        estimate_before_zupt = node._runtime_state.latest_estimate
        assert estimate_before_zupt is not None

        before_zupt_speed_mps: float = estimate_before_zupt.forward_speed_mps

        node._handle_zupt_flag(make_zupt_flag_message(True))
        node._handle_zupt(
            make_zupt_message(
                timestamp_sec=1.1,
                linear_variances_mps2=(0.05, 0.05, 0.05),
                angular_variances_rads2=(50.0, 60.0, 70.0),
            )
        )

        assert node._runtime_state.latest_estimate is not None
        assert node._runtime_state.latest_estimate.forward_speed_mps < (
            before_zupt_speed_mps
        )
        assert node._diagnostics.accepted_zupt_count == 1
        assert node._forward_twist_pub.messages[-1].twist.twist.angular.z == 0.0
    finally:
        node.stop()


def test_ahrs_speedometer_uses_linear_block_not_legacy_x_only_entry() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0, linear_accel_x=1.0))
        node._handle_imu(make_imu_message(timestamp_sec=1.0, linear_accel_x=1.0))
        node._handle_zupt_flag(make_zupt_flag_message(True))
        node._handle_zupt(
            make_zupt_message(
                timestamp_sec=1.1,
                linear_variances_mps2=(0.01, 0.25, 0.49),
                angular_variances_rads2=(0.0, 0.0, 0.0),
            )
        )

        expected_variance_mps2: float = (0.01 + 0.25 + 0.49) / 3.0
        expected_prior_variance_mps2: float = (
            1.0 + ForwardTwistConfig.forward_accel_process_variance_mps2_2
        )
        estimate = node._runtime_state.latest_estimate

        assert estimate is not None
        assert estimate.forward_speed_mps == pytest.approx(
            expected_variance_mps2
            / (expected_prior_variance_mps2 + expected_variance_mps2)
        )
        assert estimate.forward_speed_variance_mps2 == pytest.approx(
            (expected_prior_variance_mps2 * expected_variance_mps2)
            / (expected_prior_variance_mps2 + expected_variance_mps2)
        )
    finally:
        node.stop()


def make_imu_message(timestamp_sec: float, linear_accel_x: float) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    whole_seconds: int = int(timestamp_sec)
    nanoseconds: int = int((timestamp_sec - whole_seconds) * 1.0e9)
    message.header.stamp.sec = whole_seconds
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = "base_link"
    message.orientation.w = 1.0
    message.angular_velocity.x = 0.0
    message.angular_velocity.y = 0.0
    message.angular_velocity.z = 0.0
    message.linear_acceleration.x = linear_accel_x
    message.linear_acceleration.y = 0.0
    message.linear_acceleration.z = 0.0
    return message


def make_zupt_flag_message(stationary: bool) -> BoolMsg:
    message: BoolMsg = BoolMsg()
    message.data = stationary
    return message


def make_zupt_message(
    *,
    timestamp_sec: float,
    linear_variances_mps2: tuple[float, float, float],
    angular_variances_rads2: tuple[float, float, float],
) -> TwistWithCovarianceStampedMsg:
    message: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
    whole_seconds: int = int(timestamp_sec)
    nanoseconds: int = int((timestamp_sec - whole_seconds) * 1.0e9)
    message.header.stamp.sec = whole_seconds
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = "base_link"
    message.twist.twist.linear.x = 0.0
    message.twist.twist.linear.y = 0.0
    message.twist.twist.linear.z = 0.0
    message.twist.twist.angular.x = 0.0
    message.twist.twist.angular.y = 0.0
    message.twist.twist.angular.z = 0.0
    message.twist.covariance[0] = linear_variances_mps2[0]
    message.twist.covariance[7] = linear_variances_mps2[1]
    message.twist.covariance[14] = linear_variances_mps2[2]
    message.twist.covariance[21] = angular_variances_rads2[0]
    message.twist.covariance[28] = angular_variances_rads2[1]
    message.twist.covariance[35] = angular_variances_rads2[2]
    return message
