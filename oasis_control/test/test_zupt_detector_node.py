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

"""Tests for the ROS-facing stationary-twist detector node contract."""

from __future__ import annotations

from typing import Any

import pytest
import rclpy
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.nodes.zupt_detector_node import ZuptDetectorNode


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_zupt_message_publishes_full_stationary_twist_covariance() -> None:
    node: ZuptDetectorNode = ZuptDetectorNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0))
        node._handle_imu(make_imu_message(timestamp_sec=0.18))

        assert len(node._zupt_pub.messages) == 2

        zupt_message: Any = node._zupt_pub.messages[-1]
        covariance: list[float] = list(zupt_message.twist.covariance)
        linear_variance_mps2: float = (
            node._detector.state.current_linear_zupt_variance_mps2
        )
        angular_variance_rads2: float = (
            node._detector.state.current_angular_zupt_variance_rads2
        )

        assert zupt_message.twist.twist.linear.x == 0.0
        assert zupt_message.twist.twist.linear.y == 0.0
        assert zupt_message.twist.twist.linear.z == 0.0
        assert zupt_message.twist.twist.angular.x == 0.0
        assert zupt_message.twist.twist.angular.y == 0.0
        assert zupt_message.twist.twist.angular.z == 0.0
        assert covariance[0] == linear_variance_mps2
        assert covariance[7] == linear_variance_mps2
        assert covariance[14] == linear_variance_mps2
        assert covariance[21] == angular_variance_rads2
        assert covariance[28] == angular_variance_rads2
        assert covariance[35] == angular_variance_rads2
        assert covariance[1] == 0.0
        assert covariance[6] == 0.0
        assert covariance[2] == 0.0
        assert covariance[12] == 0.0
        assert covariance[8] == 0.0
        assert covariance[13] == 0.0
        assert covariance[3] == 0.0
        assert covariance[4] == 0.0
        assert covariance[5] == 0.0
        assert covariance[18] == 0.0
        assert covariance[24] == 0.0
        assert covariance[30] == 0.0
    finally:
        node.stop()


def test_zupt_message_uses_block_diagonal_stationary_twist_covariance() -> None:
    node: ZuptDetectorNode = ZuptDetectorNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0))

        assert len(node._zupt_pub.messages) == 1

        covariance: list[float] = list(node._zupt_pub.messages[-1].twist.covariance)

        assert covariance[0] > 0.0
        assert covariance[7] == covariance[0]
        assert covariance[14] == covariance[0]
        assert covariance[21] > 0.0
        assert covariance[28] == covariance[21]
        assert covariance[35] == covariance[21]
        assert covariance[15] == 0.0
        assert covariance[20] == 0.0
    finally:
        node.stop()


def make_imu_message(timestamp_sec: float) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    whole_seconds: int = int(timestamp_sec)
    nanoseconds: int = int((timestamp_sec - whole_seconds) * 1.0e9)
    message.header.stamp.sec = whole_seconds
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = "imu_link"
    message.angular_velocity.x = 0.0
    message.angular_velocity.y = 0.0
    message.angular_velocity.z = 0.0
    message.linear_acceleration.x = 0.0
    message.linear_acceleration.y = 0.0
    message.linear_acceleration.z = 0.0
    return message
