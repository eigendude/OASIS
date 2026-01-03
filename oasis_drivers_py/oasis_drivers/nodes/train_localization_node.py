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

from oasis_drivers.localization.gravity_pose_estimator import PoseEstimate
from oasis_drivers.localization.gravity_pose_estimator import TiltPoseEstimator


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "train_localization"

IMU_TOPIC = "imu"

POSE_TOPIC = "train_pose"

TILT_TOPIC = "tilt"

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

        self._estimator = TiltPoseEstimator()

        self._pose_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic=POSE_TOPIC,
            qos_profile=qos_profile,
        )

        self._tilt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic=TILT_TOPIC,
            qos_profile=qos_profile,
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Imu,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )

        self._last_imu_time: float | None = None

        self.get_logger().info("Train localization initialized")

    def stop(self) -> None:
        self.get_logger().info("Train localization deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: Imu) -> None:
        stamp = message.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if self._last_imu_time is None:
            dt_s = 0.0
        else:
            dt_s = max(0.0, timestamp - self._last_imu_time)
        self._last_imu_time = timestamp

        estimate: PoseEstimate | None = self._estimator.update(
            linear_accel=(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            angular_velocity=(
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            dt_s=dt_s,
            linear_accel_covariance=message.linear_acceleration_covariance,
            angular_velocity_covariance=message.angular_velocity_covariance,
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

        tilt_message = PoseWithCovarianceStamped()
        tilt_message.header.stamp = message.header.stamp
        tilt_message.header.frame_id = (
            message.header.frame_id if message.header.frame_id else self._frame_id
        )
        tilt_message.pose.pose.position.x = estimate.position[0]
        tilt_message.pose.pose.position.y = estimate.position[1]
        tilt_message.pose.pose.position.z = estimate.position[2]
        tilt_message.pose.pose.orientation.x = estimate.orientation[0]
        tilt_message.pose.pose.orientation.y = estimate.orientation[1]
        tilt_message.pose.pose.orientation.z = estimate.orientation[2]
        tilt_message.pose.pose.orientation.w = estimate.orientation[3]
        tilt_message.pose.covariance = estimate.covariance

        self._tilt_pub.publish(tilt_message)
