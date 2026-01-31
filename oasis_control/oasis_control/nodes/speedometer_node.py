################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS 2 node that estimates forward velocity from IMU and ZUPT updates."""

from __future__ import annotations

import math
from typing import Optional

import message_filters
import rclpy.node
import rclpy.qos
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.speedometer_core import SpeedometerCore
from oasis_control.localization.speedometer_core import SpeedometerCoreConfig
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
# Helper functions
################################################################################


def _stamp_to_sec(stamp: TimeMsg) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


################################################################################
# ROS node
################################################################################


class SpeedometerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)

        base_frame: str = str(self.get_parameter("base_frame").value)
        if not base_frame:
            self.get_logger().error("base_frame parameter is empty")
            raise RuntimeError("Missing base_frame parameter")

        self._base_frame: str = base_frame

        # Speedomter
        core_config: SpeedometerCoreConfig = SpeedometerCoreConfig()
        self._core: SpeedometerCore = SpeedometerCore(core_config)
        self._zupt_var_floor: float = core_config.zupt_var_floor
        self._bias_priors_applied: bool = False

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

    def _handle_imu_raw_with_calibration(
        self, imu_raw_msg: ImuMsg, imu_cal_msg: ImuCalibrationMsg
    ) -> None:
        timestamp_sec: float = _stamp_to_sec(imu_raw_msg.header.stamp)
        accel: float = float(imu_raw_msg.linear_acceleration.x)

        accel_var: Optional[float] = None
        accel_cov: float = float(imu_raw_msg.linear_acceleration_covariance[0])
        if math.isfinite(accel_cov) and accel_cov >= 0.0:
            accel_var = accel_cov

        if not self._bias_priors_applied:
            self._try_apply_bias_priors(imu_cal_msg)

        self._core.predict(timestamp_sec, accel, accel_var)
        self._publish_forward_twist(imu_raw_msg.header.stamp)

    def _handle_zupt(self, zupt_msg: TwistWithCovarianceStampedMsg) -> None:
        timestamp_sec: float = _stamp_to_sec(zupt_msg.header.stamp)
        zupt_var: float = float(zupt_msg.twist.covariance[0])
        if not math.isfinite(zupt_var) or zupt_var <= 0.0:
            zupt_var = self._zupt_var_floor

        self._core.apply_zupt(timestamp_sec, zupt_var)
        self._publish_forward_twist(zupt_msg.header.stamp)

    def _try_apply_bias_priors(self, imu_cal_msg: ImuCalibrationMsg) -> None:
        if not imu_cal_msg.valid:
            return

        bias_mean: float = float(imu_cal_msg.accel_bias.x)
        bias_var: float = float(imu_cal_msg.accel_param_cov[0])
        if not math.isfinite(bias_mean):
            return
        if not math.isfinite(bias_var) or bias_var <= 0.0:
            return

        self._core.set_bias_priors(bias_mean, bias_var)
        self._bias_priors_applied = True
        self.get_logger().info(
            f"Applied accel bias priors mean={bias_mean} var={bias_var}",
        )

    def _publish_forward_twist(self, stamp: TimeMsg) -> None:
        msg: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.twist.twist.linear.x = self._core.get_velocity_mps()

        covariance: list[float] = [1e6] * 36
        P_vv: float = float(self._core.get_covariance()[0, 0])
        if math.isfinite(P_vv) and P_vv >= 0.0:
            covariance[0] = P_vv
        msg.twist.covariance = covariance

        self._forward_twist_pub.publish(msg)
