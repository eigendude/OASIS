################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the ROS-facing AHRS tilt node contract."""

from __future__ import annotations

import math
from typing import Any

import pytest


rclpy = pytest.importorskip("rclpy")

from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.nodes.ahrs_tilt_node import AhrsTiltNode


UNKNOWN_COVARIANCE: list[float] = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[ImuMsg] = []

    def publish(self, message: ImuMsg) -> None:
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_level_input_publishes_level_tilt_in_base_link() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=0.0,
                )
            )
        )

        assert len(fake_pub.messages) == 1

        tilt_message: ImuMsg = fake_pub.messages[-1]
        assert tilt_message.header.frame_id == "base_link"
        assert math.isclose(tilt_message.orientation.x, 0.0, abs_tol=1.0e-9)
        assert math.isclose(tilt_message.orientation.y, 0.0, abs_tol=1.0e-9)
        assert math.isclose(tilt_message.orientation.z, 0.0, abs_tol=1.0e-9)
        assert math.isclose(tilt_message.orientation.w, 1.0, abs_tol=1.0e-9)
        assert tilt_message.orientation_covariance[8] == 1.0e6
        assert tilt_message.angular_velocity_covariance == UNKNOWN_COVARIANCE
        assert tilt_message.linear_acceleration_covariance == UNKNOWN_COVARIANCE
    finally:
        node.stop()


def test_roll_input_publishes_expected_roll_with_near_zero_pitch() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        roll_angle_rad: float = math.radians(30.0)
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=roll_angle_rad,
                    pitch_rad=0.0,
                    yaw_rad=0.0,
                )
            )
        )

        assert len(fake_pub.messages) == 1

        roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(fake_pub.messages[-1])
        assert math.isclose(roll_rad, roll_angle_rad, abs_tol=1.0e-6)
        assert math.isclose(pitch_rad, 0.0, abs_tol=1.0e-6)
        assert math.isclose(yaw_rad, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_pitch_input_publishes_expected_pitch_with_near_zero_roll() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        pitch_angle_rad: float = math.radians(20.0)
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=pitch_angle_rad,
                    yaw_rad=0.0,
                )
            )
        )

        assert len(fake_pub.messages) == 1

        roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(fake_pub.messages[-1])
        assert math.isclose(roll_rad, 0.0, abs_tol=1.0e-6)
        assert math.isclose(pitch_rad, pitch_angle_rad, abs_tol=1.0e-6)
        assert math.isclose(yaw_rad, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_yaw_only_input_publishes_level_tilt() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=math.radians(45.0),
                )
            )
        )

        assert len(fake_pub.messages) == 1

        roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(fake_pub.messages[-1])
        assert math.isclose(roll_rad, 0.0, abs_tol=1.0e-6)
        assert math.isclose(pitch_rad, 0.0, abs_tol=1.0e-6)
        assert math.isclose(yaw_rad, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_roll_pitch_tilt_is_invariant_to_input_yaw() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=math.radians(15.0),
                    pitch_rad=math.radians(-10.0),
                    yaw_rad=0.0,
                )
            )
        )
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=math.radians(15.0),
                    pitch_rad=math.radians(-10.0),
                    yaw_rad=math.radians(120.0),
                )
            )
        )

        assert len(fake_pub.messages) == 2

        first_roll_rad, first_pitch_rad, first_yaw_rad = quaternion_to_euler(
            fake_pub.messages[0]
        )
        second_roll_rad, second_pitch_rad, second_yaw_rad = quaternion_to_euler(
            fake_pub.messages[1]
        )

        assert math.isclose(first_roll_rad, second_roll_rad, abs_tol=1.0e-6)
        assert math.isclose(first_pitch_rad, second_pitch_rad, abs_tol=1.0e-6)
        assert math.isclose(first_yaw_rad, 0.0, abs_tol=1.0e-6)
        assert math.isclose(second_yaw_rad, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_wrong_frame_is_rejected() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                frame_id="imu_link",
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=0.0,
                ),
            )
        )

        assert fake_pub.messages == []
    finally:
        node.stop()


def test_yaw_covariance_remains_large_and_unobserved() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=0.0,
                )
            )
        )

        assert len(fake_pub.messages) == 1
        assert fake_pub.messages[-1].orientation_covariance[8] == 1.0e6
    finally:
        node.stop()


def test_roll_and_pitch_covariance_are_preserved_from_input() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        input_covariance: list[float] = [
            0.1,
            0.01,
            0.02,
            0.01,
            0.2,
            0.03,
            0.02,
            0.03,
            0.4,
        ]
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=math.radians(10.0),
                    pitch_rad=math.radians(-5.0),
                    yaw_rad=math.radians(15.0),
                ),
                orientation_covariance=input_covariance,
            )
        )

        assert len(fake_pub.messages) == 1
        assert fake_pub.messages[-1].orientation_covariance == [
            0.1,
            0.01,
            0.0,
            0.01,
            0.2,
            0.0,
            0.0,
            0.0,
            1.0e6,
        ]
    finally:
        node.stop()


def test_unknown_input_orientation_covariance_stays_honest() -> None:
    node: AhrsTiltNode = AhrsTiltNode()
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=0.0,
                ),
                orientation_covariance=UNKNOWN_COVARIANCE,
            )
        )

        assert len(fake_pub.messages) == 1
        assert fake_pub.messages[-1].orientation_covariance == UNKNOWN_COVARIANCE
    finally:
        node.stop()


def make_imu_message(
    *,
    frame_id: str = "base_link",
    orientation_xyzw: tuple[float, float, float, float],
    orientation_covariance: list[float] | None = None,
) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    message.header.frame_id = frame_id
    message.orientation.x = orientation_xyzw[0]
    message.orientation.y = orientation_xyzw[1]
    message.orientation.z = orientation_xyzw[2]
    message.orientation.w = orientation_xyzw[3]
    message.orientation_covariance = (
        orientation_covariance
        if orientation_covariance is not None
        else [
            0.1,
            0.0,
            0.0,
            0.0,
            0.2,
            0.0,
            0.0,
            0.0,
            0.3,
        ]
    )
    message.angular_velocity_covariance = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    message.linear_acceleration.x = 0.0
    message.linear_acceleration.y = 0.0
    message.linear_acceleration.z = 0.0
    message.linear_acceleration_covariance = [
        0.04,
        0.0,
        0.0,
        0.0,
        0.04,
        0.0,
        0.0,
        0.0,
        0.04,
    ]
    return message


def quaternion_from_roll_pitch_yaw(
    *, roll_rad: float, pitch_rad: float, yaw_rad: float
) -> tuple[float, float, float, float]:
    half_roll_rad: float = roll_rad * 0.5
    half_pitch_rad: float = pitch_rad * 0.5
    half_yaw_rad: float = yaw_rad * 0.5

    sin_half_roll: float = math.sin(half_roll_rad)
    cos_half_roll: float = math.cos(half_roll_rad)
    sin_half_pitch: float = math.sin(half_pitch_rad)
    cos_half_pitch: float = math.cos(half_pitch_rad)
    sin_half_yaw: float = math.sin(half_yaw_rad)
    cos_half_yaw: float = math.cos(half_yaw_rad)

    return (
        sin_half_roll * cos_half_pitch * cos_half_yaw
        - cos_half_roll * sin_half_pitch * sin_half_yaw,
        cos_half_roll * sin_half_pitch * cos_half_yaw
        + sin_half_roll * cos_half_pitch * sin_half_yaw,
        cos_half_roll * cos_half_pitch * sin_half_yaw
        - sin_half_roll * sin_half_pitch * cos_half_yaw,
        cos_half_roll * cos_half_pitch * cos_half_yaw
        + sin_half_roll * sin_half_pitch * sin_half_yaw,
    )


def quaternion_to_euler(message: ImuMsg) -> tuple[float, float, float]:
    qx: float = float(message.orientation.x)
    qy: float = float(message.orientation.y)
    qz: float = float(message.orientation.z)
    qw: float = float(message.orientation.w)

    sinr_cosp: float = 2.0 * (qw * qx + qy * qz)
    cosr_cosp: float = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll_rad: float = math.atan2(sinr_cosp, cosr_cosp)

    sinp: float = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch_rad: float = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch_rad = math.asin(sinp)

    siny_cosp: float = 2.0 * (qw * qz + qx * qy)
    cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_rad: float = math.atan2(siny_cosp, cosy_cosp)

    return (roll_rad, pitch_rad, yaw_rad)
