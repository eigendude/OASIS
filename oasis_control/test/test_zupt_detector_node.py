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


def test_each_published_flag_state_has_a_matching_zupt_message() -> None:
    node: ZuptDetectorNode = ZuptDetectorNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0))
        node._handle_imu(make_imu_message(timestamp_sec=0.18))
        node._handle_imu(make_imu_message(timestamp_sec=0.19, gyro_x_rads=0.2))

        assert len(node._zupt_flag_pub.messages) == 3
        assert len(node._zupt_pub.messages) == 3
        assert [message.data for message in node._zupt_flag_pub.messages] == [
            False,
            True,
            False,
        ]
    finally:
        node.stop()


def test_stationary_zupt_message_publishes_small_isotropic_covariance() -> None:
    node: ZuptDetectorNode = ZuptDetectorNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0))
        node._handle_imu(make_imu_message(timestamp_sec=0.18))

        zupt_message: Any = node._zupt_pub.messages[-1]
        covariance: list[float] = list(zupt_message.twist.covariance)
        linear_variance_mps2: float = (
            node._detector.state.current_linear_zupt_variance_mps2
        )
        angular_variance_rads2: float = (
            node._detector.state.current_angular_zupt_variance_rads2
        )

        assert node._zupt_flag_pub.messages[-1].data is True
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
        assert_block_diagonal_covariance(covariance)
    finally:
        node.stop()


def test_moving_zupt_message_publishes_large_isotropic_covariance() -> None:
    node: ZuptDetectorNode = ZuptDetectorNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0, gyro_x_rads=0.2))

        covariance: list[float] = list(node._zupt_pub.messages[-1].twist.covariance)

        assert node._zupt_flag_pub.messages[-1].data is False
        assert covariance[0] == node._detector.state.current_linear_zupt_variance_mps2
        assert covariance[7] == covariance[0]
        assert covariance[14] == covariance[0]
        assert covariance[21] == (
            node._detector.state.current_angular_zupt_variance_rads2
        )
        assert covariance[28] == covariance[21]
        assert covariance[35] == covariance[21]
        assert_block_diagonal_covariance(covariance)
    finally:
        node.stop()


def test_fast_exit_publishes_moving_pair_immediately() -> None:
    node: ZuptDetectorNode = ZuptDetectorNode()

    try:
        node._handle_imu(make_imu_message(timestamp_sec=0.0))
        node._handle_imu(make_imu_message(timestamp_sec=0.18))
        node._handle_imu(make_imu_message(timestamp_sec=0.19, gyro_x_rads=0.2))

        covariance: list[float] = list(node._zupt_pub.messages[-1].twist.covariance)

        assert node._detector.state.stationary is False
        assert node._zupt_flag_pub.messages[-1].data is False
        assert covariance[0] == node._detector.state.current_linear_zupt_variance_mps2
        assert covariance[21] == (
            node._detector.state.current_angular_zupt_variance_rads2
        )
        assert covariance[0] == node._detector._config.moving_linear_variance_mps2
        assert covariance[21] == node._detector._config.moving_angular_variance_rads2
    finally:
        node.stop()


def assert_block_diagonal_covariance(covariance: list[float]) -> None:
    linear_off_diagonal_indices: tuple[tuple[int, int], ...] = (
        (0, 1),
        (0, 2),
        (1, 2),
    )
    angular_off_diagonal_indices: tuple[tuple[int, int], ...] = (
        (3, 4),
        (3, 5),
        (4, 5),
    )
    row_index: int
    column_index: int

    for row_index, column_index in linear_off_diagonal_indices:
        assert covariance[_covariance_index(row_index, column_index)] == 0.0
        assert covariance[_covariance_index(column_index, row_index)] == 0.0

    for row_index, column_index in angular_off_diagonal_indices:
        assert covariance[_covariance_index(row_index, column_index)] == 0.0
        assert covariance[_covariance_index(column_index, row_index)] == 0.0

    for row_index in range(3):
        for column_index in range(3, 6):
            assert covariance[_covariance_index(row_index, column_index)] == 0.0
            assert covariance[_covariance_index(column_index, row_index)] == 0.0


def _covariance_index(row_index: int, column_index: int) -> int:
    return row_index * 6 + column_index


def make_imu_message(timestamp_sec: float, gyro_x_rads: float = 0.0) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    whole_seconds: int = int(timestamp_sec)
    nanoseconds: int = int((timestamp_sec - whole_seconds) * 1.0e9)
    message.header.stamp.sec = whole_seconds
    message.header.stamp.nanosec = nanoseconds
    message.header.frame_id = "imu_link"
    message.angular_velocity.x = gyro_x_rads
    message.angular_velocity.y = 0.0
    message.angular_velocity.z = 0.0
    message.linear_acceleration.x = 0.0
    message.linear_acceleration.y = 0.0
    message.linear_acceleration.z = 0.0
    return message
