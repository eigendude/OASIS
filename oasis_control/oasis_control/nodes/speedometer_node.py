################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
ROS node that estimates linear velocity from fused IMU data.
"""

from __future__ import annotations

from typing import List
from typing import Optional

import numpy as np
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

from oasis_control.localization.velocity_estimator import VelocityEstimator


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "speedometer"

# ROS topics
IMU_FUSED_TOPIC: str = "imu_fused"
TWIST_TOPIC: str = "twist"

# ROS parameters
PARAM_GRAVITY: str = "gravity_mps2"

# m/s^2 standard gravity used when subtracting gravity in world frame
DEFAULT_GRAVITY_MPS2: float = 9.80665

# s max allowed time delta before skipping propagation
MAX_DT_SPIKE_S: float = 0.2


################################################################################
# ROS node
################################################################################


class SpeedometerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """

        super().__init__(NODE_NAME)

        self.declare_parameter(PARAM_GRAVITY, DEFAULT_GRAVITY_MPS2)

        self._gravity_mps2: float = float(self.get_parameter(PARAM_GRAVITY).value)

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._twist_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStamped,
            topic=TWIST_TOPIC,
            qos_profile=qos_profile,
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Imu,
            topic=IMU_FUSED_TOPIC,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )

        self._estimator = VelocityEstimator(gravity_mps2=self._gravity_mps2)
        self._last_imu_time: Optional[float] = None
        self._last_dt_spike_log_time: Optional[float] = None

        self.get_logger().info("Speedometer initialized")

    def stop(self) -> None:
        self.get_logger().info("Speedometer deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: Imu) -> None:
        stamp: Time = message.header.stamp
        timestamp: float = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        if self._last_imu_time is None:
            self._last_imu_time = timestamp
            return

        dt_s: float = timestamp - self._last_imu_time
        self._last_imu_time = timestamp

        if dt_s <= 0.0:
            return

        if dt_s > MAX_DT_SPIKE_S:
            if (
                self._last_dt_spike_log_time is None
                or (timestamp - self._last_dt_spike_log_time) >= 1.0
            ):
                self.get_logger().warn(
                    f"Large IMU dt detected ({dt_s:.3f}s), skipping update"
                )
                self._last_dt_spike_log_time = timestamp
            dt_s = 0.0

        quat_body_to_world = np.array(
            [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ]
        )
        accel_body_mps2 = np.array(
            [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ]
        )

        cov_accel_body = np.array(
            message.linear_acceleration_covariance,
        ).reshape((3, 3))
        cov_theta_body = np.array(message.orientation_covariance).reshape((3, 3))

        v_body, cov_v_body = self._estimator.update(
            dt_s=dt_s,
            quat_body_to_world=quat_body_to_world,
            accel_body_mps2=accel_body_mps2,
            cov_accel_body=cov_accel_body,
            cov_theta_body=cov_theta_body,
        )

        twist_message = TwistWithCovarianceStamped()
        twist_message.header.stamp = message.header.stamp
        twist_message.header.frame_id = message.header.frame_id

        # Velocity is expressed in the IMU/body frame
        twist_message.twist.twist.linear.x = float(v_body[0])
        twist_message.twist.twist.linear.y = float(v_body[1])
        twist_message.twist.twist.linear.z = float(v_body[2])
        twist_message.twist.twist.angular = message.angular_velocity

        cov_angular_body = np.array(message.angular_velocity_covariance).reshape((3, 3))
        twist_message.twist.covariance = self._build_twist_covariance(
            cov_v_body=cov_v_body,
            cov_w_body=cov_angular_body,
        )

        self._twist_pub.publish(twist_message)

    @staticmethod
    def _build_twist_covariance(
        cov_v_body: np.ndarray,
        cov_w_body: np.ndarray,
    ) -> List[float]:
        covariance = np.zeros((6, 6))
        covariance[0:3, 0:3] = cov_v_body
        covariance[3:6, 3:6] = cov_w_body
        return covariance.reshape(36).tolist()
