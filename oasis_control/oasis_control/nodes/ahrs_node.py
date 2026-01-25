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
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs"

# ROS topics
IMU_RAW_TOPIC: str = "imu_raw"
IMU_CAL_TOPIC: str = "imu_calibration"
MAG_TOPIC: str = "magnetic_field"

ACCEL_UPDATE_TOPIC: str = "ahrs/updates/accel"
GYRO_UPDATE_TOPIC: str = "ahrs/updates/gyro"
MAG_UPDATE_TOPIC: str = "ahrs/updates/mag"

EXTRINSICS_T_BI_TOPIC: str = "ahrs/extrinsics/t_bi"
EXTRINSICS_T_BM_TOPIC: str = "ahrs/extrinsics/t_bm"


################################################################################
# ROS node
################################################################################


class AhrsNode(rclpy.node.Node):
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
        self._imu_raw_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuMsg,
                IMU_RAW_TOPIC,
                qos_profile=qos_profile,
            )
        )
        self._imu_cal_filter_sub: message_filters.Subscriber = (
            message_filters.Subscriber(
                self,
                ImuCalibrationMsg,
                IMU_CAL_TOPIC,
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
        if (
            imu_msg.header.stamp.sec != cal_msg.header.stamp.sec
            or imu_msg.header.stamp.nanosec != cal_msg.header.stamp.nanosec
        ):
            self.get_logger().warning(
                "IMU sample and calibration have mismatched timestamps"
            )
            return

        # TODO

    def _handle_mag(self, message: MagneticFieldMsg) -> None:
        # TODO
        pass
