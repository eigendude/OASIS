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
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg

from oasis_msgs.msg import ConductorState as ConductorStateMsg
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "zupt_detector"

# ROS topics
CONDUCTOR_STATE_TOPIC: str = "conductor_state"
IMU_CAL_TOPIC: str = "imu_calibration"
TILT_TOPIC = "tilt"
ZUPT_FLAG_TOPIC = "zupt_flag"
ZUPT_TOPIC = "zupt"

################################################################################
# ROS node
################################################################################


class ZuptDetectorNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources
        """

        super().__init__(NODE_NAME)

        # QoS profile
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS Publishers
        self._zupt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStamped,
            topic=ZUPT_TOPIC,
            qos_profile=qos_profile,
        )
        self._zupt_flag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=BoolMsg,
            topic=ZUPT_FLAG_TOPIC,
            qos_profile=qos_profile,
        )

        # ZUPT detector
        # TODO

        # ROS Subscribers
        self._conductor_state_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ConductorStateMsg,
                topic=CONDUCTOR_STATE_TOPIC,
                callback=self._handle_conductor_state,
                qos_profile=qos_profile,
            )
        )
        self._imu_calibration_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ImuCalibrationMsg,
                topic=IMU_CAL_TOPIC,
                callback=self._handle_imu_calibration,
                qos_profile=qos_profile,
            )
        )
        self._tilt_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=TILT_TOPIC,
            callback=self._handle_tilt,
            qos_profile=qos_profile,
        )

        self.get_logger().info("ZUPT detector node initialized")

    def stop(self) -> None:
        self.get_logger().info("ZUPT detector node deinitialized")

        self.destroy_node()
