################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

from oasis_drivers.localization.gravity_pose_estimator import GravityPoseEstimator
from oasis_drivers.localization.gravity_pose_estimator import PoseEstimate


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "train_localization"

IMU_TOPIC = "imu"

POSE_TOPIC = "train_pose"

DEFAULT_FRAME_ID = "falcon/base_link"


################################################################################
# ROS node
################################################################################


class TrainLocalizationNode(rclpy.node.Node):
    def __init__(self) -> None:
        """Initialize resources."""
        super().__init__(NODE_NAME)

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._frame_id: str = DEFAULT_FRAME_ID

        self._estimator = GravityPoseEstimator()

        self._pose_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic=POSE_TOPIC,
            qos_profile=qos_profile,
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Imu,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )

        self.get_logger().info("Train localization initialized")

    def stop(self) -> None:
        self.get_logger().info("Train localization deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: Imu) -> None:
        estimate: PoseEstimate | None = self._estimator.update(
            linear_accel=(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            linear_accel_covariance=message.linear_acceleration_covariance,
        )

        if estimate is None:
            return

        pose_message = PoseWithCovarianceStamped()
        pose_message.header.stamp = message.header.stamp
        pose_message.header.frame_id = (
            message.header.frame_id if message.header.frame_id else self._frame_id
        )

        pose_message.pose.pose.position.x = estimate.position[0]
        pose_message.pose.pose.position.y = estimate.position[1]
        pose_message.pose.pose.position.z = estimate.position[2]

        pose_message.pose.pose.orientation.x = estimate.orientation[0]
        pose_message.pose.pose.orientation.y = estimate.orientation[1]
        pose_message.pose.pose.orientation.z = estimate.orientation[2]
        pose_message.pose.pose.orientation.w = estimate.orientation[3]

        pose_message.pose.covariance = estimate.covariance

        self._pose_pub.publish(pose_message)
