################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

# mypy: disable-error-code=attr-defined

"""Tests for AHRS speedometer ROS validation and message mapping."""

from __future__ import annotations

from typing import Any

import pytest
import rclpy
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg

from oasis_control.nodes.ahrs_speedometer_node import AhrsSpeedometerNode
from oasis_control.nodes.zupt_detector_node import _build_zupt_covariance


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_node_waits_for_zupt_then_publishes_full_mapped_twist() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        node._handle_imu(make_imu(1.0))
        assert node._forward_twist_pub.messages == []

        flag: BoolMsg = BoolMsg()
        flag.data = True
        node._handle_zupt_flag(flag)
        node._handle_zupt(make_zupt(1.1))
        node._handle_imu(make_imu(1.1, angular=(1.0, 2.0, 3.0)))

        output: Any = node._forward_twist_pub.messages[-1]
        assert output.header.frame_id == "base_link"
        assert output.header.stamp.sec == 1
        assert output.header.stamp.nanosec == 100_000_000
        assert output.twist.twist.linear.y == 0.0
        assert output.twist.twist.linear.z == 0.0
        assert output.twist.twist.angular.x != 0.0
        assert output.twist.twist.angular.y != 0.0
        assert output.twist.twist.angular.z != 0.0
        assert len(output.twist.covariance) == 36
        assert output.twist.covariance[1] == 0.0
        assert output.twist.covariance[6] == 0.0
        assert output.twist.covariance[22] != 0.0
    finally:
        node.stop()


def test_speedometer_endpoints_use_bounded_reliable_qos() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        assert_reliable_measurement_qos(node._forward_twist_pub.qos_profile)
        assert_reliable_measurement_qos(node._imu_sub.qos_profile)
        assert_reliable_measurement_qos(node._zupt_sub.qos_profile)
        flag_qos: Any = node._zupt_flag_sub.qos_profile
        assert flag_qos.reliability == rclpy.qos.QoSReliabilityPolicy.RELIABLE
        assert flag_qos.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        assert flag_qos.history == rclpy.qos.QoSHistoryPolicy.KEEP_LAST
        assert flag_qos.depth == 1
    finally:
        node.stop()


def test_next_imu_publishes_complete_raw_angular_covariance() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        node._handle_imu(make_imu(1.1, angular=(1.0, 2.0, 3.0)))

        output: Any = node._forward_twist_pub.messages[-1]
        assert output.twist.twist.angular.x == 1.0
        assert output.twist.twist.angular.y == 2.0
        assert output.twist.twist.angular.z == 3.0
        expected_angular_covariance: list[float] = [
            0.2,
            0.01,
            0.02,
            0.01,
            0.3,
            0.03,
            0.02,
            0.03,
            0.4,
        ]
        actual_angular_covariance: list[float] = [
            output.twist.covariance[(row + 3) * 6 + column + 3]
            for row in range(3)
            for column in range(3)
        ]
        assert actual_angular_covariance == expected_angular_covariance
        assert output.twist.twist.linear.y == 0.0
        assert output.twist.twist.linear.z == 0.0
        assert output.header.frame_id == "base_link"
    finally:
        node.stop()


def test_hud_fields_contain_signed_speed_and_its_variance() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        message: ImuMsg = make_imu(1.1)
        message.linear_acceleration.x = -2.0
        node._handle_imu(message)

        output: Any = node._forward_twist_pub.messages[-1]
        hud_speed: float = output.twist.twist.linear.x
        hud_speed_variance: float = output.twist.covariance[0]
        assert hud_speed == pytest.approx(-0.2)
        assert hud_speed_variance > 0.0
    finally:
        node.stop()


@pytest.mark.parametrize(
    "mutation",
    ["frame", "quaternion", "measurement", "orientation_cov", "angular_cov"],
)
def test_node_rejects_invalid_imu_contract(mutation: str) -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        message: ImuMsg = make_imu(1.0)
        if mutation == "frame":
            message.header.frame_id = "imu_link"
        elif mutation == "quaternion":
            message.orientation.w = 0.0
        elif mutation == "measurement":
            message.linear_acceleration.x = float("nan")
        elif mutation == "orientation_cov":
            message.orientation_covariance[1] = 1.0
        else:
            message.angular_velocity_covariance[0] = -0.1

        node._handle_imu(message)
        assert node._forward_twist_pub.messages == []
        assert ("warning", "Rejected invalid AHRS IMU sample") in (
            node.get_logger().messages
        )
    finally:
        node.stop()


def test_unavailable_covariances_are_accepted_with_fallbacks() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        flag: BoolMsg = BoolMsg()
        flag.data = True
        node._handle_zupt_flag(flag)
        node._handle_zupt(make_zupt(1.0))
        message: ImuMsg = make_imu(1.0)
        message.orientation_covariance[0] = -1.0
        message.angular_velocity_covariance[0] = -1.0
        message.linear_acceleration_covariance[0] = -1.0

        node._handle_imu(message)

        assert len(node._forward_twist_pub.messages) == 1
        assert any(
            level == "warning" and "fallback" in text
            for level, text in node.get_logger().messages
        )
    finally:
        node.stop()


def test_repeated_acceleration_fallback_logs_only_transitions() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        for timestamp_sec in (1.1, 1.2, 1.3):
            message: ImuMsg = make_imu(timestamp_sec)
            message.linear_acceleration_covariance[0] = -1.0
            node._handle_imu(message)

        fallback_warnings: list[str] = [
            text
            for level, text in node.get_logger().messages
            if level == "warning" and "fallback" in text
        ]
        assert fallback_warnings == ["Using acceleration covariance fallback"]

        node._handle_imu(make_imu(1.4))
        resumed_messages: list[str] = [
            text
            for level, text in node.get_logger().messages
            if level == "info" and "resumed" in text
        ]
        assert resumed_messages == ["Measured acceleration covariance resumed"]
    finally:
        node.stop()


def test_angular_covariance_fallback_logs_each_transition_once() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        for timestamp_sec in (1.1, 1.2, 1.3):
            message: ImuMsg = make_imu(timestamp_sec)
            message.angular_velocity_covariance[0] = -1.0
            node._handle_imu(message)

        assert messages_containing(
            node, "warning", "angular-velocity covariance fallback"
        ) == ["Using angular-velocity covariance fallback"]

        node._handle_imu(make_imu(1.4))
        assert messages_containing(
            node, "info", "angular-velocity covariance resumed"
        ) == ["Measured angular-velocity covariance resumed"]

        unavailable_again: ImuMsg = make_imu(1.5)
        unavailable_again.angular_velocity_covariance[0] = -1.0
        node._handle_imu(unavailable_again)
        assert messages_containing(
            node, "warning", "angular-velocity covariance fallback"
        ) == [
            "Using angular-velocity covariance fallback",
            "Using angular-velocity covariance fallback",
        ]
    finally:
        node.stop()


def test_acceleration_and_angular_fallback_transitions_are_independent() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        acceleration_fallback: ImuMsg = make_imu(1.1)
        acceleration_fallback.linear_acceleration_covariance[0] = -1.0
        node._handle_imu(acceleration_fallback)
        assert len(messages_containing(node, "warning", "acceleration")) == 1
        assert messages_containing(node, "warning", "angular-velocity") == []

        angular_fallback: ImuMsg = make_imu(1.2)
        angular_fallback.angular_velocity_covariance[0] = -1.0
        node._handle_imu(angular_fallback)
        assert len(messages_containing(node, "info", "acceleration")) == 1
        assert len(messages_containing(node, "warning", "angular-velocity")) == 1
    finally:
        node.stop()


def test_repeated_valid_imu_samples_do_not_emit_per_sample_debug_logs() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        for timestamp_sec in (1.1, 1.2, 1.3, 1.4):
            node._handle_imu(make_imu(timestamp_sec))

        assert messages_containing(node, "debug", "Accepted IMU:") == []
    finally:
        node.stop()


def test_initialization_success_is_logged_exactly_once() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        node._handle_imu(make_imu(1.1))
        node._handle_imu(make_imu(1.2))

        assert messages_containing(
            node, "info", "initialized from ZUPT; publishing forward twist"
        ) == ["AHRS speedometer initialized from ZUPT; publishing forward twist"]
    finally:
        node.stop()


def test_publication_resumes_after_excessive_timestamp_gap() -> None:
    node: AhrsSpeedometerNode = initialized_node(1.0)
    try:
        gap_message: ImuMsg = make_imu(2.0)
        gap_message.linear_acceleration.x = 10.0
        node._handle_imu(gap_message)
        assert len(node._forward_twist_pub.messages) == 1
        assert node._speedometer.last_rejection == "excessive_gap"

        node._handle_imu(make_imu(2.1))
        assert len(node._forward_twist_pub.messages) == 2
        assert node._forward_twist_pub.messages[-1].twist.twist.linear.x == 0.0
    finally:
        node.stop()


def test_real_zupt_covariance_initializes_speedometer() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        flag: BoolMsg = BoolMsg()
        flag.data = True
        node._handle_zupt_flag(flag)
        zupt: TwistWithCovarianceStampedMsg = make_zupt(1.0)
        stationary_variance: float = 0.06**2
        zupt.twist.covariance = _build_zupt_covariance(
            linear_variance_mps2=stationary_variance,
            angular_variance_rads2=stationary_variance,
        )
        node._handle_zupt(zupt)
        node._handle_imu(make_imu(1.0))

        assert node._speedometer.initialized is True
        assert len(node._forward_twist_pub.messages) == 1
        assert [zupt.twist.covariance[index] for index in (0, 7, 14, 21, 28, 35)] == [
            stationary_variance
        ] * 6
    finally:
        node.stop()


@pytest.mark.parametrize(
    ("quaternion", "valid"),
    [
        ((0.0, 0.0, 0.0, 1.0), True),
        ((0.0, 0.0, 0.0, 2.0), True),
        ((0.0, 0.0, 0.0, 0.0), False),
        ((0.0, 0.0, 0.0, 1.0e-13), False),
        ((float("nan"), 0.0, 0.0, 1.0), False),
        ((float("inf"), 0.0, 0.0, 1.0), False),
    ],
)
def test_quaternion_validation_normalizes_finite_nonzero_inputs(
    quaternion: tuple[float, float, float, float], valid: bool
) -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        message: ImuMsg = make_imu(1.0)
        message.orientation.x = quaternion[0]
        message.orientation.y = quaternion[1]
        message.orientation.z = quaternion[2]
        message.orientation.w = quaternion[3]

        assert (node._parse_imu(message) is not None) is valid
    finally:
        node.stop()


def test_output_frame_parameter_is_removed() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        assert "output_frame_id" not in node._parameters
        assert node._base_frame_id == "base_link"
    finally:
        node.stop()


def test_imu_link_zupt_from_detector_initializes_speedometer() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        flag: BoolMsg = BoolMsg()
        flag.data = True
        node._handle_zupt_flag(flag)
        detector_zupt: TwistWithCovarianceStampedMsg = make_zupt(1.0)
        detector_zupt.header.frame_id = "imu_link"
        node._handle_zupt(detector_zupt)
        node._handle_imu(make_imu(1.0))

        assert node._speedometer.initialized is True
        assert len(node._forward_twist_pub.messages) == 1
        assert node._forward_twist_pub.messages[0].header.frame_id == "base_link"
    finally:
        node.stop()


@pytest.mark.parametrize("frame_id", ["imu_link", "another_imu", ""])
def test_valid_zupt_frame_id_is_not_inspected(frame_id: str) -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        zupt: TwistWithCovarianceStampedMsg = make_zupt(1.0)
        zupt.header.frame_id = frame_id
        node._handle_zupt(zupt)

        assert node._speedometer._latest_zupt is not None
        assert node._speedometer.last_zupt_status == "accepted"
    finally:
        node.stop()


def test_zupt_correction_is_identical_for_different_frame_ids() -> None:
    outputs: list[tuple[list[float], list[float]]] = []
    for frame_id in ("imu_link", "unrelated_frame"):
        node: AhrsSpeedometerNode = AhrsSpeedometerNode()
        try:
            flag: BoolMsg = BoolMsg()
            flag.data = True
            node._handle_zupt_flag(flag)
            zupt: TwistWithCovarianceStampedMsg = make_zupt(1.0)
            zupt.header.frame_id = frame_id
            node._handle_zupt(zupt)
            node._handle_imu(make_imu(1.0))
            output: Any = node._forward_twist_pub.messages[-1]
            outputs.append(
                (
                    [
                        output.twist.twist.linear.x,
                        output.twist.twist.angular.x,
                        output.twist.twist.angular.y,
                        output.twist.twist.angular.z,
                    ],
                    list(output.twist.covariance),
                )
            )
        finally:
            node.stop()

    assert outputs[0] == outputs[1]


def test_invalid_zupt_covariance_is_not_stored() -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        invalid_covariance: TwistWithCovarianceStampedMsg = make_zupt(1.0)
        invalid_covariance.twist.covariance[1] = 1.0
        node._handle_zupt(invalid_covariance)
        assert node._speedometer._latest_zupt is None
        assert node._speedometer.last_zupt_status == "invalid_input_covariance"
    finally:
        node.stop()


@pytest.mark.parametrize(
    ("mean_index", "covariance_index", "value"),
    [
        (0, None, 0.01),
        (None, 7, 2.0),
        (None, 28, 2.0),
        (None, 1, 0.1),
        (None, 22, 0.1),
        (None, 3, 0.1),
        (None, 0, 0.0),
        (None, 0, float("nan")),
    ],
)
def test_non_invariant_zupt_shape_is_rejected(
    mean_index: int | None, covariance_index: int | None, value: float
) -> None:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    try:
        message: TwistWithCovarianceStampedMsg = make_zupt(1.0)
        if mean_index is not None:
            message.twist.twist.linear.x = value
        if covariance_index is not None:
            message.twist.covariance[covariance_index] = value

        node._handle_zupt(message)

        assert node._speedometer._latest_zupt is None
        assert node._speedometer.last_zupt_status == "invalid_input_covariance"
    finally:
        node.stop()


def make_imu(
    timestamp_sec: float,
    angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    set_stamp(message, timestamp_sec)
    message.header.frame_id = "base_link"
    message.orientation.w = 1.0
    message.angular_velocity.x = angular[0]
    message.angular_velocity.y = angular[1]
    message.angular_velocity.z = angular[2]
    message.linear_acceleration.x = 0.0
    message.linear_acceleration.y = 0.0
    message.linear_acceleration.z = 0.0
    message.orientation_covariance = diagonal_covariance_3(0.01)
    message.angular_velocity_covariance = [
        0.2,
        0.01,
        0.02,
        0.01,
        0.3,
        0.03,
        0.02,
        0.03,
        0.4,
    ]
    message.linear_acceleration_covariance = diagonal_covariance_3(0.5)
    return message


def make_zupt(timestamp_sec: float) -> TwistWithCovarianceStampedMsg:
    message: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
    set_stamp(message, timestamp_sec)
    message.header.frame_id = "base_link"
    for index in range(6):
        message.twist.covariance[index * 6 + index] = 0.01
    return message


def diagonal_covariance_3(variance: float) -> list[float]:
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def set_stamp(message: Any, timestamp_sec: float) -> None:
    whole_seconds: int = int(timestamp_sec)
    message.header.stamp.sec = whole_seconds
    message.header.stamp.nanosec = round((timestamp_sec - whole_seconds) * 1.0e9)


def initialized_node(timestamp_sec: float) -> AhrsSpeedometerNode:
    node: AhrsSpeedometerNode = AhrsSpeedometerNode()
    flag: BoolMsg = BoolMsg()
    flag.data = True
    node._handle_zupt_flag(flag)
    node._handle_zupt(make_zupt(timestamp_sec))
    node._handle_imu(make_imu(timestamp_sec))
    return node


def messages_containing(node: AhrsSpeedometerNode, level: str, text: str) -> list[str]:
    return [
        message
        for message_level, message in node.get_logger().messages
        if message_level == level and text in message
    ]


def assert_reliable_measurement_qos(qos_profile: Any) -> None:
    assert qos_profile.reliability == rclpy.qos.QoSReliabilityPolicy.RELIABLE
    assert qos_profile.durability == rclpy.qos.QoSDurabilityPolicy.VOLATILE
    assert qos_profile.history == rclpy.qos.QoSHistoryPolicy.KEEP_LAST
    assert qos_profile.depth == 50
