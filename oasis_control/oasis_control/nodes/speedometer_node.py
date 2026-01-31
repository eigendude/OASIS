################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


import message_filters
import rclpy.node
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg

from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "speedometer"

# ROS topics
FORWARD_TWIST_TOPIC: str = "forward_twist"
IMU_CAL_TOPIC: str = "imu_calibration"
IMU_RAW_TOPIC: str = "imu_raw"
ZUPT_TOPIC: str = "zupt"

# Default base frame identifier
DEFAULT_BASE_FRAME: str = "base_link"


################################################################################
# ROS node
################################################################################


class SpeedometerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources
        """

        super().__init__(NODE_NAME)

        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)

        base_frame: str = str(self.get_parameter("base_frame").value)
        if not base_frame:
            self.get_logger().error("base_frame parameter is empty")
            raise RuntimeError("Missing base_frame parameter")

        self._base_frame: str = base_frame

        # Velocity estimator
        # TODO

        # QoS profiles
        sensor_data_qos: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS Publishers
        self._forward_twist_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=FORWARD_TWIST_TOPIC,
            qos_profile=sensor_data_qos,
        )

        # ROS Subscribers
        self._imu_cal_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuCalibrationMsg,
                IMU_CAL_TOPIC,
                qos_profile=sensor_data_qos,
            )
        )
        self._imu_raw_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuMsg,
                IMU_RAW_TOPIC,
                qos_profile=sensor_data_qos,
            )
        )
        self._zupt_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=ZUPT_TOPIC,
            callback=self._handle_zupt,
            qos_profile=sensor_data_qos,
        )

        # ROS message synchronizers
        self._imu_sync: message_filters.TimeSynchronizer = (
            message_filters.TimeSynchronizer(
                [self._imu_raw_filter_sub, self._imu_cal_filter_sub],
                queue_size=20,
            )
        )
        self._imu_sync.registerCallback(self._handle_imu_raw_with_calibration)

        self.get_logger().info("Speedometer node initialized")

    def stop(self) -> None:
        self.get_logger().info("Speedometer node deinitialized")

        self.destroy_node()
