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
from nav_msgs.msg import Odometry as OdometryMsg
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

# ROS parameters and defaults
PARAM_WORLD_FRAME_ID: str = "world_frame_id"
PARAM_ODOM_FRAME_ID: str = "odom_frame_id"
PARAM_BODY_FRAME_ID: str = "body_frame_id"

DEFAULT_WORLD_FRAME_ID: str = "world"
DEFAULT_ODOM_FRAME_ID: str = "odom"
DEFAULT_BODY_FRAME_ID: str = "base_link"

# TODO: AHRS parameters and defaults
PARAM_GRAVITY_MPS2: str = "gravity_mps2"

DEFAULT_GRAVITY_MPS2: float = 9.80665


################################################################################
# Utilities
################################################################################


# Nanoseconds per second for converting ROS params to AHRS nanoseconds
NS_PER_S: int = 1_000_000_000


# Convert seconds to nanoseconds
def _ns_from_seconds(seconds: float) -> int:
    return int(round(seconds * NS_PER_S))


################################################################################
# ROS node
################################################################################


class AhrsNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources
        """

        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter(PARAM_WORLD_FRAME_ID, DEFAULT_WORLD_FRAME_ID)
        self.declare_parameter(PARAM_ODOM_FRAME_ID, DEFAULT_ODOM_FRAME_ID)
        self.declare_parameter(PARAM_BODY_FRAME_ID, DEFAULT_BODY_FRAME_ID)

        # TODO: AHRS parameters
        self.declare_parameter(PARAM_GRAVITY_MPS2, DEFAULT_GRAVITY_MPS2)

        # Load parameters
        self._world_frame_id: str = str(self.get_parameter(PARAM_WORLD_FRAME_ID).value)
        self._odom_frame_id: str = str(self.get_parameter(PARAM_ODOM_FRAME_ID).value)
        self._body_frame_id: str = str(self.get_parameter(PARAM_BODY_FRAME_ID).value)

        # TODO: AHRS parameters
        self._gravity_mps2: float = float(self.get_parameter(PARAM_GRAVITY_MPS2).value)

        # QoS profile
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS Publishers
        # TODO

        # ROS Subscribers
        self._mag_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=MagneticFieldMsg,
            topic=MAG_TOPIC,
            callback=self._handle_mag,
            qos_profile=qos_profile,
        )
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

        # ROS message synchronizers
        self._imu_sync: message_filters.TimeSynchronizer = (
            message_filters.TimeSynchronizer(
                [self._imu_raw_filter_sub, self._imu_cal_filter_sub],
                queue_size=20,
            )
        )
        self._imu_sync.registerCallback(self._handle_imu_raw_with_calibration)

        # TODO: AHRS parameters

        # TODO: State

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
