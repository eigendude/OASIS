################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Shared test helpers for AHRS node-runtime tests."""

from __future__ import annotations

from typing import Any

from geometry_msgs.msg import (
    AccelWithCovarianceStamped as AccelWithCovarianceStampedMsg,
)
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import make_mounting_transform
from oasis_control.nodes.ahrs_node import AhrsNode
from oasis_msgs.msg import AhrsStatus as AhrsStatusMsg


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class FakeTransformBroadcaster:
    def __init__(self) -> None:
        self.transforms: list[list[TransformStampedMsg]] = []

    def sendTransform(self, transforms: Any) -> None:
        if isinstance(transforms, list):
            self.transforms.append(list(transforms))
            return

        self.transforms.append([transforms])


def make_cached_mounting_transform(
    quaternion_xyzw: tuple[float, float, float, float],
) -> MountingTransform:
    return make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=quaternion_xyzw,
    )


def make_imu_message(
    *,
    frame_id: str = "imu_link",
    quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    timestamp_ns: int = 1_000_000_000,
    orientation_covariance: list[float] | None = None,
    linear_acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0),
    linear_acceleration_covariance: list[float] | None = None,
) -> ImuMsg:
    message: ImuMsg = ImuMsg()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    message.orientation.x = quaternion_xyzw[0]
    message.orientation.y = quaternion_xyzw[1]
    message.orientation.z = quaternion_xyzw[2]
    message.orientation.w = quaternion_xyzw[3]
    message.orientation_covariance = (
        orientation_covariance
        if orientation_covariance is not None
        else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    message.angular_velocity.x = 0.1
    message.angular_velocity.y = 0.2
    message.angular_velocity.z = 0.3
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
    message.linear_acceleration.x = linear_acceleration[0]
    message.linear_acceleration.y = linear_acceleration[1]
    message.linear_acceleration.z = linear_acceleration[2]
    message.linear_acceleration_covariance = (
        linear_acceleration_covariance
        if linear_acceleration_covariance is not None
        else [
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
    )
    return message


def make_accel_message(
    *,
    frame_id: str = "imu_link",
    accel_vector: tuple[float, float, float] = (0.0, 0.0, -9.81),
    timestamp_ns: int = 950_000_000,
    covariance: list[float] | None = None,
) -> AccelWithCovarianceStampedMsg:
    message: AccelWithCovarianceStampedMsg = AccelWithCovarianceStampedMsg()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
    message.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
    message.accel.accel.linear.x = accel_vector[0]
    message.accel.accel.linear.y = accel_vector[1]
    message.accel.accel.linear.z = accel_vector[2]
    message.accel.covariance = (
        covariance
        if covariance is not None
        else [
            0.04,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.04,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.04,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    )
    return message


def make_gravity_message(
    *,
    frame_id: str = "imu_link",
    gravity_vector: tuple[float, float, float] = (0.0, 0.0, -9.81),
    timestamp_ns: int = 900_000_000,
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
        else [
            0.04,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.04,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.04,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    )
    return message


def make_node(
    mounting_transform: MountingTransform | None = None,
) -> tuple[
    AhrsNode,
    FakePublisher,
    FakePublisher,
    FakePublisher,
    FakePublisher,
    FakePublisher,
    FakeTransformBroadcaster,
]:
    fake_diag_pub: FakePublisher = FakePublisher()
    fake_accel_pub: FakePublisher = FakePublisher()
    fake_gravity_pub: FakePublisher = FakePublisher()
    fake_imu_pub: FakePublisher = FakePublisher()
    fake_odom_pub: FakePublisher = FakePublisher()
    fake_tf_broadcaster: FakeTransformBroadcaster = FakeTransformBroadcaster()

    node: AhrsNode = AhrsNode(
        tf_broadcaster=fake_tf_broadcaster,
        enable_status_timer=False,
    )
    node._mounting_transform = mounting_transform
    node._diagnostics.has_mounting = mounting_transform is not None
    node._diag_pub = fake_diag_pub
    node._accel_pub = fake_accel_pub
    node._gravity_pub = fake_gravity_pub
    node._imu_pub = fake_imu_pub
    node._odom_pub = fake_odom_pub
    return (
        node,
        fake_diag_pub,
        fake_accel_pub,
        fake_gravity_pub,
        fake_imu_pub,
        fake_odom_pub,
        fake_tf_broadcaster,
    )


def last_diag(diag_pub: FakePublisher) -> AhrsStatusMsg:
    message: AhrsStatusMsg = diag_pub.messages[-1]
    return message
