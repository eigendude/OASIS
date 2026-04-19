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

"""Tests for the ROS-facing AHRS tilt node contract."""

from __future__ import annotations

import math
from typing import Any

import pytest
import rclpy
from geometry_msgs.msg import (
    AccelWithCovarianceStamped as AccelWithCovarianceStampedMsg,
)
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from sensor_msgs.msg import Imu as ImuMsg
from tf2_ros import TransformException

from oasis_control.localization.common.measurements.tilt_covariance import (
    gravity_covariance_to_tilt_variance_rad2,
)
from oasis_control.nodes.ahrs_tilt_node import AhrsTiltNode


UNKNOWN_COVARIANCE: list[float] = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[ImuMsg] = []

    def publish(self, message: ImuMsg) -> None:
        self.messages.append(message)


class FakeTfBuffer:
    def __init__(self, transform_message: TransformStampedMsg | None) -> None:
        self._transform_message: TransformStampedMsg | None = transform_message

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: Any
    ) -> TransformStampedMsg:
        if self._transform_message is None:
            raise TransformException(
                f"missing transform {target_frame} <- {source_frame}"
            )

        return self._transform_message


@pytest.fixture(scope="module", autouse=True)
def rclpy_context() -> Any:
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_tilt_mean_still_comes_from_ahrs_orientation() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_gravity(
            make_gravity_message(
                gravity_vector=(3.0, 1.0, -9.2),
                covariance=gravity_covariance_3x3_to_row_major(
                    (
                        (0.04, 0.0, 0.0),
                        (0.0, 0.04, 0.0),
                        (0.0, 0.0, 0.04),
                    )
                ),
            )
        )
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=math.radians(15.0),
                    pitch_rad=math.radians(-10.0),
                    yaw_rad=math.radians(75.0),
                )
            )
        )

        assert len(fake_pub.messages) == 1

        roll_rad: float
        pitch_rad: float
        yaw_rad: float
        roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(fake_pub.messages[-1])
        assert math.isclose(roll_rad, math.radians(15.0), abs_tol=1.0e-6)
        assert math.isclose(pitch_rad, math.radians(-10.0), abs_tol=1.0e-6)
        assert math.isclose(yaw_rad, 0.0, abs_tol=1.0e-6)
    finally:
        node.stop()


def test_tilt_covariance_comes_from_gravity_not_full_ahrs_covariance() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    gravity_covariance: tuple[tuple[float, float, float], ...] = (
        (0.04, 0.0, 0.0),
        (0.0, 0.01, 0.0),
        (0.0, 0.0, 0.09),
    )
    expected_variance_rad2: float = gravity_covariance_to_tilt_variance_rad2(
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_mps2_2=gravity_covariance,
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                covariance=gravity_covariance_3x3_to_row_major(gravity_covariance)
            )
        )
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=math.radians(20.0),
                    pitch_rad=math.radians(-5.0),
                    yaw_rad=math.radians(30.0),
                ),
                orientation_covariance=[
                    10.0,
                    1.0,
                    2.0,
                    1.0,
                    20.0,
                    3.0,
                    2.0,
                    3.0,
                    40.0,
                ],
            )
        )

        assert len(fake_pub.messages) == 1
        assert math.isclose(
            fake_pub.messages[-1].orientation_covariance[0],
            expected_variance_rad2,
            abs_tol=1.0e-12,
        )
        assert math.isclose(
            fake_pub.messages[-1].orientation_covariance[4],
            expected_variance_rad2,
            abs_tol=1.0e-12,
        )
        assert fake_pub.messages[-1].orientation_covariance[1] == 0.0
        assert fake_pub.messages[-1].orientation_covariance[3] == 0.0
    finally:
        node.stop()


def test_yaw_covariance_remains_large_and_unobserved() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_gravity(make_gravity_message())
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


def test_missing_gravity_publishes_unknown_covariance() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_imu(
            make_imu_message(
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=math.radians(5.0),
                    pitch_rad=math.radians(3.0),
                    yaw_rad=math.radians(45.0),
                )
            )
        )

        assert len(fake_pub.messages) == 1
        assert fake_pub.messages[-1].orientation_covariance == UNKNOWN_COVARIANCE
    finally:
        node.stop()


def test_stale_gravity_publishes_unknown_covariance() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    try:
        node._handle_gravity(make_gravity_message(timestamp_ns=0))
        node._handle_imu(
            make_imu_message(
                timestamp_ns=1_000_000_000,
                orientation_xyzw=quaternion_from_roll_pitch_yaw(
                    roll_rad=0.0,
                    pitch_rad=0.0,
                    yaw_rad=0.0,
                ),
            )
        )

        assert len(fake_pub.messages) == 1
        assert fake_pub.messages[-1].orientation_covariance == UNKNOWN_COVARIANCE
    finally:
        node.stop()


def test_level_case_covariance_matches_gravity_tilt_scale() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    gravity_covariance: tuple[tuple[float, float, float], ...] = (
        (0.04, 0.0, 0.0),
        (0.0, 0.04, 0.0),
        (0.0, 0.0, 0.04),
    )
    expected_variance_rad2: float = gravity_covariance_to_tilt_variance_rad2(
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_mps2_2=gravity_covariance,
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                covariance=gravity_covariance_3x3_to_row_major(gravity_covariance)
            )
        )
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
        assert math.isclose(
            fake_pub.messages[-1].orientation_covariance[0],
            expected_variance_rad2,
            abs_tol=1.0e-12,
        )
        assert math.isclose(
            fake_pub.messages[-1].orientation_covariance[4],
            expected_variance_rad2,
            abs_tol=1.0e-12,
        )
    finally:
        node.stop()


def test_gravity_covariance_is_rotated_into_base_link_before_scaling() -> None:
    node: AhrsTiltNode = AhrsTiltNode(
        tf_buffer=FakeTfBuffer(make_mounting_transform(yaw_rad=math.radians(45.0)))
    )
    fake_pub: FakePublisher = FakePublisher()
    node._tilt_pub = fake_pub

    gravity_covariance_imu: tuple[tuple[float, float, float], ...] = (
        (0.09, 0.0, 0.0),
        (0.0, 0.01, 0.0),
        (0.0, 0.0, 0.01),
    )
    rotated_gravity_covariance_base: tuple[tuple[float, float, float], ...] = (
        (0.05, 0.04, 0.0),
        (0.04, 0.05, 0.0),
        (0.0, 0.0, 0.01),
    )
    expected_variance_rad2: float = gravity_covariance_to_tilt_variance_rad2(
        gravity_mps2=(0.0, 0.0, -9.81),
        gravity_covariance_mps2_2=rotated_gravity_covariance_base,
    )

    try:
        node._handle_gravity(
            make_gravity_message(
                covariance=gravity_covariance_3x3_to_row_major(gravity_covariance_imu)
            )
        )
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
        assert math.isclose(
            fake_pub.messages[-1].orientation_covariance[0],
            expected_variance_rad2,
            abs_tol=1.0e-12,
        )
        assert math.isclose(
            fake_pub.messages[-1].orientation_covariance[4],
            expected_variance_rad2,
            abs_tol=1.0e-12,
        )
    finally:
        node.stop()


def test_wrong_imu_frame_is_rejected() -> None:
    node: AhrsTiltNode = AhrsTiltNode(tf_buffer=FakeTfBuffer(make_mounting_transform()))
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


def make_mounting_transform(yaw_rad: float = 0.0) -> TransformStampedMsg:
    transform_message: TransformStampedMsg = TransformStampedMsg()
    transform_message.header.frame_id = "base_link"
    transform_message.child_frame_id = "imu_link"
    quaternion_xyzw: tuple[float, float, float, float] = quaternion_from_roll_pitch_yaw(
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=yaw_rad,
    )
    transform_message.transform.rotation.x = quaternion_xyzw[0]
    transform_message.transform.rotation.y = quaternion_xyzw[1]
    transform_message.transform.rotation.z = quaternion_xyzw[2]
    transform_message.transform.rotation.w = quaternion_xyzw[3]
    return transform_message


def make_gravity_message(
    *,
    frame_id: str = "imu_link",
    gravity_vector: tuple[float, float, float] = (0.0, 0.0, -9.81),
    timestamp_ns: int = 950_000_000,
    covariance: list[float] | None = None,
) -> AccelWithCovarianceStampedMsg:
    message: AccelWithCovarianceStampedMsg = AccelWithCovarianceStampedMsg()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    message.accel.accel.linear.x = gravity_vector[0]
    message.accel.accel.linear.y = gravity_vector[1]
    message.accel.accel.linear.z = gravity_vector[2]
    message.accel.covariance = (
        covariance
        if covariance is not None
        else gravity_covariance_3x3_to_row_major(
            (
                (0.04, 0.0, 0.0),
                (0.0, 0.04, 0.0),
                (0.0, 0.0, 0.04),
            )
        )
    )
    return message


def make_imu_message(
    *,
    frame_id: str = "base_link",
    orientation_xyzw: tuple[float, float, float, float],
    timestamp_ns: int = 1_000_000_000,
    orientation_covariance: list[float] | None = None,
) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    message.orientation.x = orientation_xyzw[0]
    message.orientation.y = orientation_xyzw[1]
    message.orientation.z = orientation_xyzw[2]
    message.orientation.w = orientation_xyzw[3]
    message.orientation_covariance = (
        orientation_covariance
        if orientation_covariance is not None
        else [
            0.1,
            0.01,
            0.02,
            0.01,
            0.2,
            0.03,
            0.02,
            0.03,
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


def gravity_covariance_3x3_to_row_major(
    covariance_3x3: tuple[tuple[float, float, float], ...],
) -> list[float]:
    row_major_covariance: list[float] = [0.0] * 36
    row_major_covariance[0] = covariance_3x3[0][0]
    row_major_covariance[1] = covariance_3x3[0][1]
    row_major_covariance[2] = covariance_3x3[0][2]
    row_major_covariance[6] = covariance_3x3[1][0]
    row_major_covariance[7] = covariance_3x3[1][1]
    row_major_covariance[8] = covariance_3x3[1][2]
    row_major_covariance[12] = covariance_3x3[2][0]
    row_major_covariance[13] = covariance_3x3[2][1]
    row_major_covariance[14] = covariance_3x3[2][2]
    return row_major_covariance


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
    quaternion_x: float = float(message.orientation.x)
    quaternion_y: float = float(message.orientation.y)
    quaternion_z: float = float(message.orientation.z)
    quaternion_w: float = float(message.orientation.w)

    sin_roll_cos_pitch: float = 2.0 * (
        quaternion_w * quaternion_x + quaternion_y * quaternion_z
    )
    cos_roll_cos_pitch: float = 1.0 - 2.0 * (
        quaternion_x * quaternion_x + quaternion_y * quaternion_y
    )
    roll_rad: float = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch: float = 2.0 * (quaternion_w * quaternion_y - quaternion_z * quaternion_x)
    clamped_sin_pitch: float = max(-1.0, min(1.0, sin_pitch))
    pitch_rad: float = math.asin(clamped_sin_pitch)

    sin_yaw_cos_pitch: float = 2.0 * (
        quaternion_w * quaternion_z + quaternion_x * quaternion_y
    )
    cos_yaw_cos_pitch: float = 1.0 - 2.0 * (
        quaternion_y * quaternion_y + quaternion_z * quaternion_z
    )
    yaw_rad: float = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)

    return (roll_rad, pitch_rad, yaw_rad)
