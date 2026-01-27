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
import rclpy.qos
import rclpy.subscription
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_mounting"

# ROS topics
AHRS_MOUNT_TOPIC: str = "ahrs_mount"
AHRS_MOUNT_DIAGNOSTICS_TOPIC: str = "ahrs_mount_diagnostics"
AHRS_MOUNT_TRANSFORM_TOPIC: str = "ahrs_mount_transform"
IMU_CAL_TOPIC: str = "imu_calibration"
IMU_RAW_TOPIC: str = "imu_raw"
MAG_TOPIC: str = "magnetic_field"

# TF topics
# TODO


################################################################################
# ROS node
################################################################################


class AhrsMountingNode(rclpy.node.Node):
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
        # TODO

        # AHRS filter
        # TODO

        # ROS Subscribers
        # TODO: AHRS topics
        self._imu_cal_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuCalibrationMsg,
                IMU_CAL_TOPIC,
                qos_profile=qos_profile,
            )
        )
        self._imu_raw_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuMsg,
                IMU_RAW_TOPIC,
                qos_profile=qos_profile,
            )
        )
        self._mag_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=MagneticFieldMsg,
            topic=MAG_TOPIC,
            callback=self._handle_mag,
            qos_profile=qos_profile,
        )

        # ROS message synchronizers
        self._imu_sync: message_filters.TimeSynchronizer = (
            message_filters.TimeSynchronizer(
                [self._imu_raw_filter_sub, self._imu_cal_filter_sub],
                queue_size=20,
            )
        )
        self._imu_sync.registerCallback(self._handle_imu_raw_with_calibration)

        self.get_logger().info("AHRS node initialized")

    def stop(self) -> None:
        self.get_logger().info("AHRS node deinitialized")

        self.destroy_node()

    def _handle_imu_raw_with_calibration(
        self, imu_msg: ImuMsg, cal_msg: ImuCalibrationMsg
    ) -> None:
        # TODO
        pass

    def _handle_mag(self, message: MagneticFieldMsg) -> None:
        # TODO
        pass
