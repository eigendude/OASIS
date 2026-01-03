################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math

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

        self._estimator = GravityPoseEstimator()

        self._pose_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic=POSE_TOPIC,
            qos_profile=qos_profile,
        )

        self._tilt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Imu,
            topic=TILT_TOPIC,
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

        imu_cov = list(message.linear_acceleration_covariance)
        accel_cov_unknown = self._covariance_unknown(imu_cov)
        if accel_cov_unknown and imu_cov[0] >= 0.0:
            imu_cov = [-1.0] + [0.0] * 8

        gyro_cov = list(message.angular_velocity_covariance)
        gyro_cov_unknown = self._covariance_unknown(gyro_cov)
        if gyro_cov_unknown:
            gyro_cov = [-1.0] + [0.0] * 8

        pose_cov = estimate.covariance

        # Map pose covariance [x,y,z,roll,pitch,yaw] to Imu orientation entries
        var_roll = self._clamp_variance(pose_cov[21], 0.0)
        var_pitch = self._clamp_variance(pose_cov[28], 0.0)
        var_yaw = self._clamp_variance(pose_cov[35], 1.0e6)
        var_yaw = max(var_yaw, 1.0e6)
        cov_rp = self._clamp_covariance(pose_cov[22])

        tilt_message = Imu()
        tilt_message.header.stamp = message.header.stamp
        tilt_message.header.frame_id = (
            message.header.frame_id if message.header.frame_id else self._frame_id
        )

        tilt_message.orientation.x = estimate.orientation[0]
        tilt_message.orientation.y = estimate.orientation[1]
        tilt_message.orientation.z = estimate.orientation[2]
        tilt_message.orientation.w = estimate.orientation[3]
        tilt_message.orientation_covariance = [
            var_roll,
            cov_rp,
            0.0,
            cov_rp,
            var_pitch,
            0.0,
            0.0,
            0.0,
            var_yaw,
        ]

        # Unknown covariances use -1.0 in the first element per ROS conventions
        tilt_message.angular_velocity.x = message.angular_velocity.x
        tilt_message.angular_velocity.y = message.angular_velocity.y
        tilt_message.angular_velocity.z = message.angular_velocity.z
        tilt_message.angular_velocity_covariance = gyro_cov

        tilt_message.linear_acceleration.x = message.linear_acceleration.x
        tilt_message.linear_acceleration.y = message.linear_acceleration.y
        tilt_message.linear_acceleration.z = message.linear_acceleration.z
        tilt_message.linear_acceleration_covariance = imu_cov

        self._tilt_pub.publish(tilt_message)

    @staticmethod
    def _covariance_unknown(covariance: list[float]) -> bool:
        if len(covariance) != 9:
            return True

        if covariance[0] < 0.0:
            return True

        return all(value == 0.0 for value in covariance)

    @staticmethod
    def _clamp_variance(value: float, default: float) -> float:
        if not math.isfinite(value):
            return default

        return max(value, 0.0)

    @staticmethod
    def _clamp_covariance(value: float) -> float:
        if not math.isfinite(value):
            return 0.0

        return value
