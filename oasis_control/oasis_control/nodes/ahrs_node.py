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
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import MagneticField as MagneticFieldMsg

from oasis_control.localization.ahrs.ahrs_clock import RosAhrsClock
from oasis_control.localization.ahrs.ahrs_conversions import ahrs_time_from_ros
from oasis_control.localization.ahrs.ahrs_conversions import cov3x3_from_ros
from oasis_control.localization.ahrs.ahrs_conversions import vector3_from_ros
from oasis_control.localization.ahrs.ahrs_filter import AhrsFilter
from oasis_control.localization.ahrs.ahrs_params import declare_ahrs_params
from oasis_control.localization.ahrs.ahrs_params import load_ahrs_config
from oasis_control.localization.ahrs.ahrs_publishers import AhrsPublishers
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsEventType
from oasis_control.localization.ahrs.ahrs_types import AhrsImuPacket
from oasis_control.localization.ahrs.ahrs_types import ImuCalibrationData
from oasis_control.localization.ahrs.ahrs_types import ImuSample
from oasis_control.localization.ahrs.ahrs_types import MagSample


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

        # Declare parameters
        declare_ahrs_params(self)
        config = load_ahrs_config(self)

        # QoS profile
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS Publishers
        self._publishers: AhrsPublishers = AhrsPublishers(
            self, config, qos_profile
        )

        # AHRS filter
        self._filter: AhrsFilter = AhrsFilter(
            config=config, clock=RosAhrsClock(self)
        )

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

        try:
            imu_sample: ImuSample = ImuSample(
                frame_id=imu_msg.header.frame_id,
                angular_velocity_rps=vector3_from_ros(imu_msg.angular_velocity),
                linear_acceleration_mps2=vector3_from_ros(
                    imu_msg.linear_acceleration
                ),
                angular_velocity_cov=cov3x3_from_ros(
                    imu_msg.angular_velocity_covariance
                ),
                linear_acceleration_cov=cov3x3_from_ros(
                    imu_msg.linear_acceleration_covariance
                ),
            )
            calibration: ImuCalibrationData = ImuCalibrationData(
                valid=bool(cal_msg.valid),
                frame_id=cal_msg.header.frame_id,
                accel_bias_mps2=vector3_from_ros(cal_msg.accel_bias),
                accel_a=[float(value) for value in cal_msg.accel_a],
                accel_param_cov=[float(value) for value in cal_msg.accel_param_cov],
                gyro_bias_rps=vector3_from_ros(cal_msg.gyro_bias),
                gyro_bias_cov=cov3x3_from_ros(cal_msg.gyro_bias_cov),
                gravity_mps2=float(cal_msg.gravity_mps2),
                fit_sample_count=int(cal_msg.fit_sample_count),
                rms_residual_mps2=float(cal_msg.rms_residual_mps2),
                temperature_c=float(cal_msg.temperature_c),
                temperature_var_c2=float(cal_msg.temperature_var_c2),
            )
        except ValueError as exc:
            self.get_logger().warning(f"Invalid IMU covariance data: {exc}")
            return

        packet: AhrsImuPacket = AhrsImuPacket(
            imu=imu_sample,
            calibration=calibration,
        )
        event: AhrsEvent = AhrsEvent(
            t_meas=ahrs_time_from_ros(imu_msg.header.stamp),
            topic=IMU_RAW_TOPIC,
            frame_id=imu_msg.header.frame_id,
            event_type=AhrsEventType.IMU,
            payload=packet,
        )

        outputs = self._filter.handle_event(event)
        self._publishers.publish_outputs(outputs)

    def _handle_mag(self, message: MagneticFieldMsg) -> None:
        try:
            sample: MagSample = MagSample(
                frame_id=message.header.frame_id,
                magnetic_field_t=vector3_from_ros(message.magnetic_field),
                magnetic_field_cov=cov3x3_from_ros(
                    message.magnetic_field_covariance
                ),
            )
        except ValueError as exc:
            self.get_logger().warning(f"Invalid magnetometer covariance data: {exc}")
            return

        event: AhrsEvent = AhrsEvent(
            t_meas=ahrs_time_from_ros(message.header.stamp),
            topic=MAG_TOPIC,
            frame_id=message.header.frame_id,
            event_type=AhrsEventType.MAG,
            payload=sample,
        )

        outputs = self._filter.handle_event(event)
        self._publishers.publish_outputs(outputs)
