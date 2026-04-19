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

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.ahrs_tilt_estimator import AhrsTiltEstimate
from oasis_control.localization.ahrs_tilt_estimator import AhrsTiltEstimator
from oasis_control.localization.common.algebra.covariance import (
    UNKNOWN_ORIENTATION_COVARIANCE,
)
from oasis_control.localization.common.algebra.covariance import parse_row_major_matrix3


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_tilt"

# ROS topics
IMU_TOPIC: str = "imu"
TILT_TOPIC: str = "tilt"

# Frame contract
BASE_FRAME_ID: str = "base_link"

# sensor_msgs/Imu covariance policy for unestimated fields
UNKNOWN_COVARIANCE: list[float] = list(UNKNOWN_ORIENTATION_COVARIANCE)


################################################################################
# ROS node
################################################################################


class AhrsTiltNode(rclpy.node.Node):
    """
    Publish a roll/pitch-only view of mounted AHRS orientation.

    The input IMU is expected to already be mounted into `base_link`. The
    output quaternion preserves roll and pitch from the AHRS orientation while
    suppressing yaw in the published tilt output.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """

        super().__init__(NODE_NAME)

        self._estimator: AhrsTiltEstimator = AhrsTiltEstimator()

        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._tilt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ImuMsg,
            topic=TILT_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_qos_profile,
        )

        self.get_logger().info("AHRS tilt node initialized")

    def stop(self) -> None:
        self.get_logger().info("AHRS tilt node deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: ImuMsg) -> None:
        if message.header.frame_id != BASE_FRAME_ID:
            self.get_logger().warning(
                "Ignoring AHRS tilt sample with unexpected frame_id "
                f"'{message.header.frame_id}', expected '{BASE_FRAME_ID}'"
            )
            return

        orientation_covariance_unknown: bool = message.orientation_covariance[0] == -1.0
        orientation_covariance_rad2 = None
        if not orientation_covariance_unknown:
            orientation_covariance_rad2 = parse_row_major_matrix3(
                message.orientation_covariance
            )

        tilt_estimate: AhrsTiltEstimate | None = self._estimator.update(
            orientation_xyzw=(
                float(message.orientation.x),
                float(message.orientation.y),
                float(message.orientation.z),
                float(message.orientation.w),
            ),
            orientation_covariance_rad2=orientation_covariance_rad2,
            orientation_covariance_unknown=orientation_covariance_unknown,
        )
        if tilt_estimate is None:
            return

        tilt_message: ImuMsg = ImuMsg()
        tilt_message.header = message.header
        tilt_message.header.frame_id = BASE_FRAME_ID

        tilt_message.orientation.x = tilt_estimate.quaternion_xyzw[0]
        tilt_message.orientation.y = tilt_estimate.quaternion_xyzw[1]
        tilt_message.orientation.z = tilt_estimate.quaternion_xyzw[2]
        tilt_message.orientation.w = tilt_estimate.quaternion_xyzw[3]
        tilt_message.orientation_covariance = tilt_estimate.orientation_covariance

        tilt_message.angular_velocity.x = 0.0
        tilt_message.angular_velocity.y = 0.0
        tilt_message.angular_velocity.z = 0.0
        tilt_message.angular_velocity_covariance = list(UNKNOWN_COVARIANCE)

        tilt_message.linear_acceleration.x = 0.0
        tilt_message.linear_acceleration.y = 0.0
        tilt_message.linear_acceleration.z = 0.0
        tilt_message.linear_acceleration_covariance = list(UNKNOWN_COVARIANCE)

        self._tilt_pub.publish(tilt_message)
