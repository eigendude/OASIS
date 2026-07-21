################################################################################
#
#  Copyright (C) 2026 Garrett Brown
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
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg


################################################################################
# ROS parameters
################################################################################


# Default node name
NODE_NAME: str = "ahrs_speedometer"

# ROS topics
FORWARD_TWIST_TOPIC: str = "forward_twist"
IMU_TOPIC: str = "imu"
ZUPT_FLAG_TOPIC: str = "zupt_flag"
ZUPT_TOPIC: str = "zupt"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_OUTPUT_FRAME_ID: str = "output_frame_id"

DEFAULT_BASE_FRAME_ID: str = "base_link"
DEFAULT_OUTPUT_FRAME_ID: str = "base_link"


################################################################################
# ROS node
################################################################################


class AhrsSpeedometerNode(rclpy.node.Node):
    """
    ROS node for the AHRS speedometer pipeline.

    The published twist remains on the `ahrs/forward_twist` topic in
    `base_link` for now, but the motion model is the flat-surface
    `body_gravity` contract from the design doc rather than a free 3D
    body-axis estimate.

    Incoming ZUPT messages carry a full 6D stationary body-twist measurement.
    This node consumes only the scalar zero-speed correction derived from the
    leading `3 x 3` linear covariance block and does not consume the angular
    stationary-twist block directly in this pass.
    """

    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        self.declare_parameter(PARAM_OUTPUT_FRAME_ID, DEFAULT_OUTPUT_FRAME_ID)

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._output_frame_id: str = str(
            self.get_parameter(PARAM_OUTPUT_FRAME_ID).value
        )

        # ROS QoS profiles
        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS publishers
        self._forward_twist_pub: rclpy.publisher.Publisher[
            TwistWithCovarianceStampedMsg
        ] = self.create_publisher(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=FORWARD_TWIST_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        # ROS subscribers
        self._imu_sub: rclpy.subscription.Subscription[ImuMsg] = (
            self.create_subscription(
                msg_type=ImuMsg,
                topic=IMU_TOPIC,
                callback=self._handle_imu,
                qos_profile=sensor_qos_profile,
            )
        )
        self._zupt_sub: rclpy.subscription.Subscription[
            TwistWithCovarianceStampedMsg
        ] = self.create_subscription(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=ZUPT_TOPIC,
            callback=self._handle_zupt,
            qos_profile=sensor_qos_profile,
        )
        self._zupt_flag_sub: rclpy.subscription.Subscription[BoolMsg] = (
            self.create_subscription(
                msg_type=BoolMsg,
                topic=ZUPT_FLAG_TOPIC,
                callback=self._handle_zupt_flag,
                qos_profile=sensor_qos_profile,
            )
        )

        self.get_logger().info("AHRS speedometer node initialized")

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("AHRS speedometer node deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: ImuMsg) -> None:
        # TODO
        pass

    def _handle_zupt(self, message: TwistWithCovarianceStampedMsg) -> None:
        # TODO
        pass

    def _handle_zupt_flag(self, message: BoolMsg) -> None:
        # TODO
        pass
