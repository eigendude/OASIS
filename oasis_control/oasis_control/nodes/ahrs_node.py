################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.timer
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    AccelWithCovarianceStamped as AccelWithCovarianceStampedMsg,
)
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from nav_msgs.msg import Odometry as OdometryMsg
from sensor_msgs.msg import Imu as ImuMsg
from tf2_ros import TransformBroadcaster

from oasis_msgs.msg import AhrsStatus as AhrsStatusMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_node"

# ROS topics
GRAVITY_TOPIC: str = "gravity"
IMU_TOPIC: str = "imu"
OUTPUT_DIAG_TOPIC: str = "ahrs/diag"
OUTPUT_IMU_TOPIC: str = "ahrs/imu"
OUTPUT_ODOM_TOPIC: str = "ahrs/odom"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_IMU_FRAME_ID: str = "imu_frame_id"
PARAM_ODOM_FRAME_ID: str = "odom_frame_id"
PARAM_WORLD_FRAME_ID: str = "world_frame_id"

DEFAULT_BASE_FRAME_ID: str = "base_link"
DEFAULT_IMU_FRAME_ID: str = "imu_link"
DEFAULT_ODOM_FRAME_ID: str = "odom"
DEFAULT_WORLD_FRAME_ID: str = "world"

# Timer period for publishing placeholder TF edges and runtime diagnostics
#
# Units: s
#
STATUS_TIMER_PERIOD_SEC: float = 0.1


################################################################################
# ROS node
################################################################################


@dataclass
class AhrsDiagnosticsState:
    accepted_imu_count: int = 0
    accepted_gravity_count: int = 0
    rejected_imu_count: int = 0
    rejected_gravity_count: int = 0
    has_gravity: bool = False
    has_mounting: bool = False
    last_bad_imu_frame_id: str = ""
    last_bad_gravity_frame_id: str = ""


class AhrsNode(rclpy.node.Node):
    """
    Skeleton ROS node for fusing IMU and gravity into an AHRS estimate
    """

    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        self.declare_parameter(PARAM_IMU_FRAME_ID, DEFAULT_IMU_FRAME_ID)
        self.declare_parameter(PARAM_ODOM_FRAME_ID, DEFAULT_ODOM_FRAME_ID)
        self.declare_parameter(PARAM_WORLD_FRAME_ID, DEFAULT_WORLD_FRAME_ID)

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._imu_frame_id: str = str(self.get_parameter(PARAM_IMU_FRAME_ID).value)
        self._odom_frame_id: str = str(self.get_parameter(PARAM_ODOM_FRAME_ID).value)
        self._world_frame_id: str = str(self.get_parameter(PARAM_WORLD_FRAME_ID).value)

        # AHRS state
        self._diagnostics: AhrsDiagnosticsState = AhrsDiagnosticsState()

        # ROS QoS profiles
        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS publishers
        self._diag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=AhrsStatusMsg,
            topic=OUTPUT_DIAG_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._imu_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ImuMsg,
            topic=OUTPUT_IMU_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._odom_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=OdometryMsg,
            topic=OUTPUT_ODOM_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        # ROS subscribers
        self._gravity_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=AccelWithCovarianceStampedMsg,
            topic=GRAVITY_TOPIC,
            callback=self._handle_gravity,
            qos_profile=sensor_qos_profile,
        )
        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_qos_profile,
        )

        # ROS TF broadcaster
        self._tf_broadcaster: TransformBroadcaster = TransformBroadcaster(self)

        # ROS timers
        self._status_timer: rclpy.timer.Timer = self.create_timer(
            STATUS_TIMER_PERIOD_SEC,
            self._publish_runtime_outputs,
        )

        # Publish initial message
        self._publish_runtime_outputs()

        self.get_logger().info("AHRS node initialized")

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("AHRS node deinitialized")

        self.destroy_node()

    def _handle_gravity(self, message: AccelWithCovarianceStampedMsg) -> None:
        if message.header.frame_id != self._imu_frame_id:
            self._diagnostics.rejected_gravity_count += 1
            self._diagnostics.last_bad_gravity_frame_id = message.header.frame_id
            self._publish_runtime_outputs()
            return

        self._diagnostics.accepted_gravity_count += 1
        self._diagnostics.has_gravity = True
        self._diagnostics.last_bad_gravity_frame_id = ""

        self._publish_runtime_outputs()

    def _handle_imu(self, message: ImuMsg) -> None:
        if message.header.frame_id != self._imu_frame_id:
            self._diagnostics.rejected_imu_count += 1
            self._diagnostics.last_bad_imu_frame_id = message.header.frame_id
            self._publish_runtime_outputs()
            return

        self._diagnostics.accepted_imu_count += 1
        self._diagnostics.last_bad_imu_frame_id = ""

        self._publish_runtime_outputs()

    def _publish_runtime_outputs(self) -> None:
        self._publish_dummy_tf()
        self._publish_status()

    def _publish_dummy_tf(self) -> None:
        stamp: TimeMsg = self.get_clock().now().to_msg()

        world_to_odom: TransformStampedMsg = self._make_identity_transform(
            parent_frame_id=self._world_frame_id,
            child_frame_id=self._odom_frame_id,
            stamp=stamp,
        )
        odom_to_base: TransformStampedMsg = self._make_identity_transform(
            parent_frame_id=self._odom_frame_id,
            child_frame_id=self._base_frame_id,
            stamp=stamp,
        )

        self._tf_broadcaster.sendTransform([world_to_odom, odom_to_base])

    def _publish_status(self) -> None:
        status_message: AhrsStatusMsg = AhrsStatusMsg()
        status_message.header.stamp = self.get_clock().now().to_msg()
        status_message.header.frame_id = self._world_frame_id
        status_message.status = self._compute_status_code()
        status_message.accepted_imu_count = self._diagnostics.accepted_imu_count
        status_message.accepted_gravity_count = self._diagnostics.accepted_gravity_count
        status_message.rejected_imu_count = self._diagnostics.rejected_imu_count
        status_message.rejected_gravity_count = self._diagnostics.rejected_gravity_count
        status_message.gravity_residual_norm = math.nan
        status_message.gravity_mahalanobis_distance = math.nan
        status_message.has_gravity = self._diagnostics.has_gravity
        status_message.has_mounting = self._diagnostics.has_mounting
        status_message.status_text = self._compute_status_text()

        self._diag_pub.publish(status_message)

    def _compute_status_code(self) -> int:
        if self._diagnostics.last_bad_imu_frame_id:
            return AhrsStatusMsg.STATUS_BAD_IMU_FRAME

        if self._diagnostics.last_bad_gravity_frame_id:
            return AhrsStatusMsg.STATUS_BAD_GRAVITY_FRAME

        if self._diagnostics.accepted_imu_count == 0:
            return AhrsStatusMsg.STATUS_WAITING_FOR_IMU

        if self._diagnostics.accepted_gravity_count == 0:
            return AhrsStatusMsg.STATUS_WAITING_FOR_GRAVITY

        if not self._diagnostics.has_mounting:
            return AhrsStatusMsg.STATUS_MOUNTING_UNAVAILABLE

        return AhrsStatusMsg.STATUS_OK

    def _compute_status_text(self) -> str:
        if self._diagnostics.last_bad_imu_frame_id:
            return (
                "Received IMU frame "
                f"'{self._diagnostics.last_bad_imu_frame_id}' but expected "
                f"'{self._imu_frame_id}'"
            )

        if self._diagnostics.last_bad_gravity_frame_id:
            return (
                "Received gravity frame "
                f"'{self._diagnostics.last_bad_gravity_frame_id}' but expected "
                f"'{self._imu_frame_id}'"
            )

        if self._diagnostics.accepted_imu_count == 0:
            return "Waiting for IMU samples"

        if self._diagnostics.accepted_gravity_count == 0:
            return "Waiting for gravity samples"

        if not self._diagnostics.has_mounting:
            return "Waiting for mounting transform implementation"

        return "AHRS runtime inputs available"

    def _make_identity_transform(
        self,
        parent_frame_id: str,
        child_frame_id: str,
        stamp: TimeMsg,
    ) -> TransformStampedMsg:
        transform_message: TransformStampedMsg = TransformStampedMsg()
        transform_message.header.stamp = stamp
        transform_message.header.frame_id = parent_frame_id
        transform_message.child_frame_id = child_frame_id
        transform_message.transform.translation.x = 0.0
        transform_message.transform.translation.y = 0.0
        transform_message.transform.translation.z = 0.0
        transform_message.transform.rotation.x = 0.0
        transform_message.transform.rotation.y = 0.0
        transform_message.transform.rotation.z = 0.0
        transform_message.transform.rotation.w = 1.0

        return transform_message
