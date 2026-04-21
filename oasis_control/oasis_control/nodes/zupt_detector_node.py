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

import math

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg

from oasis_control.localization.zupt_detector import ZuptDecision
from oasis_control.localization.zupt_detector import ZuptDetector
from oasis_control.localization.zupt_detector import ZuptDetectorConfig


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "zupt_detector"

# ROS topics
IMU_TOPIC: str = "imu"
ZUPT_FLAG_TOPIC: str = "zupt_flag"
ZUPT_TOPIC: str = "zupt"

PARAM_GYRO_ENTER_THRESHOLD_RADS: str = "gyro_enter_threshold_rads"
PARAM_GYRO_EXIT_THRESHOLD_RADS: str = "gyro_exit_threshold_rads"
PARAM_ACCEL_ENTER_THRESHOLD_MPS2: str = "accel_enter_threshold_mps2"
PARAM_ACCEL_EXIT_THRESHOLD_MPS2: str = "accel_exit_threshold_mps2"
PARAM_MIN_STATIONARY_SEC: str = "min_stationary_sec"
PARAM_MIN_MOVING_SEC: str = "min_moving_sec"
PARAM_STATIONARY_LINEAR_VELOCITY_SIGMA_MPS: str = "stationary_linear_velocity_sigma_mps"
PARAM_STATIONARY_ANGULAR_VELOCITY_SIGMA_RADS: str = (
    "stationary_angular_velocity_sigma_rads"
)
PARAM_MOVING_LINEAR_VARIANCE_MPS2: str = "moving_linear_variance_mps2"
PARAM_MOVING_ANGULAR_VARIANCE_RADS2: str = "moving_angular_variance_rads2"

LINEAR_COVARIANCE_DIAGONAL_INDICES: tuple[int, int, int] = (0, 7, 14)
ANGULAR_COVARIANCE_DIAGONAL_INDICES: tuple[int, int, int] = (21, 28, 35)


################################################################################
# ROS node
################################################################################


class ZuptDetectorNode(rclpy.node.Node):
    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        detector_config: ZuptDetectorConfig = self._read_detector_config()
        self._detector: ZuptDetector = ZuptDetector(detector_config)

        # QoS profiles
        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS Publishers
        self._zupt_flag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=BoolMsg,
            topic=ZUPT_FLAG_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._zupt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=ZUPT_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        # ROS Subscribers
        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_qos_profile,
        )

        self.get_logger().info(
            "ZUPT detector node initialized "
            f"(gyro enter={detector_config.gyro_enter_threshold_rads:.3f} rad/s, "
            f"accel enter={detector_config.accel_enter_threshold_mps2:.3f} m/s^2, "
            f"stationary linear sigma="
            f"{detector_config.stationary_linear_velocity_sigma_mps:.3f} m/s)"
        )

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("ZUPT detector node deinitialized")

        self.destroy_node()

    def _read_detector_config(self) -> ZuptDetectorConfig:
        self.declare_parameter(
            PARAM_GYRO_ENTER_THRESHOLD_RADS,
            ZuptDetectorConfig.gyro_enter_threshold_rads,
        )
        self.declare_parameter(
            PARAM_GYRO_EXIT_THRESHOLD_RADS,
            ZuptDetectorConfig.gyro_exit_threshold_rads,
        )
        self.declare_parameter(
            PARAM_ACCEL_ENTER_THRESHOLD_MPS2,
            ZuptDetectorConfig.accel_enter_threshold_mps2,
        )
        self.declare_parameter(
            PARAM_ACCEL_EXIT_THRESHOLD_MPS2,
            ZuptDetectorConfig.accel_exit_threshold_mps2,
        )
        self.declare_parameter(
            PARAM_MIN_STATIONARY_SEC,
            ZuptDetectorConfig.min_stationary_sec,
        )
        self.declare_parameter(
            PARAM_MIN_MOVING_SEC,
            ZuptDetectorConfig.min_moving_sec,
        )
        self.declare_parameter(
            PARAM_STATIONARY_LINEAR_VELOCITY_SIGMA_MPS,
            ZuptDetectorConfig.stationary_linear_velocity_sigma_mps,
        )
        self.declare_parameter(
            PARAM_MOVING_LINEAR_VARIANCE_MPS2,
            ZuptDetectorConfig.moving_linear_variance_mps2,
        )
        self.declare_parameter(
            PARAM_STATIONARY_ANGULAR_VELOCITY_SIGMA_RADS,
            ZuptDetectorConfig.stationary_angular_velocity_sigma_rads,
        )
        self.declare_parameter(
            PARAM_MOVING_ANGULAR_VARIANCE_RADS2,
            ZuptDetectorConfig.moving_angular_variance_rads2,
        )

        config: ZuptDetectorConfig = ZuptDetectorConfig(
            gyro_enter_threshold_rads=float(
                self.get_parameter(PARAM_GYRO_ENTER_THRESHOLD_RADS).value
            ),
            gyro_exit_threshold_rads=float(
                self.get_parameter(PARAM_GYRO_EXIT_THRESHOLD_RADS).value
            ),
            accel_enter_threshold_mps2=float(
                self.get_parameter(PARAM_ACCEL_ENTER_THRESHOLD_MPS2).value
            ),
            accel_exit_threshold_mps2=float(
                self.get_parameter(PARAM_ACCEL_EXIT_THRESHOLD_MPS2).value
            ),
            min_stationary_sec=float(
                self.get_parameter(PARAM_MIN_STATIONARY_SEC).value
            ),
            min_moving_sec=float(self.get_parameter(PARAM_MIN_MOVING_SEC).value),
            stationary_linear_velocity_sigma_mps=float(
                self.get_parameter(PARAM_STATIONARY_LINEAR_VELOCITY_SIGMA_MPS).value
            ),
            moving_linear_variance_mps2=float(
                self.get_parameter(PARAM_MOVING_LINEAR_VARIANCE_MPS2).value
            ),
            stationary_angular_velocity_sigma_rads=float(
                self.get_parameter(PARAM_STATIONARY_ANGULAR_VELOCITY_SIGMA_RADS).value
            ),
            moving_angular_variance_rads2=float(
                self.get_parameter(PARAM_MOVING_ANGULAR_VARIANCE_RADS2).value
            ),
        )
        return config

    def _handle_imu(self, message: ImuMsg) -> None:
        timestamp_sec: float = _time_to_float_sec(message.header.stamp)
        angular_velocity_rads: tuple[float, float, float] = (
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        )
        linear_accel_mps2: tuple[float, float, float] = (
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        )
        decision: ZuptDecision | None = self._detector.update(
            timestamp_sec=timestamp_sec,
            angular_velocity_rads=angular_velocity_rads,
            linear_accel_mps2=linear_accel_mps2,
        )
        if decision is None:
            return

        self.get_logger().debug(
            "ZUPT "
            f"stationary={decision.stationary} "
            f"reason={decision.reason} "
            f"gyro_norm={decision.gyro_norm_rads:.4f} "
            f"accel_norm={decision.accel_norm_mps2:.4f} "
            f"linear_var={decision.linear_zupt_variance_mps2:.6f} "
            f"angular_var={decision.angular_zupt_variance_rads2:.6f} "
            f"enter_dwell={decision.enter_dwell_sec:.3f} "
            f"exit_dwell={decision.exit_dwell_sec:.3f}"
        )

        flag_message: BoolMsg = BoolMsg()
        flag_message.data = decision.stationary
        self._zupt_flag_pub.publish(flag_message)

        zupt_message: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
        zupt_message.header = message.header
        zupt_message.twist.twist.linear.x = 0.0
        zupt_message.twist.twist.linear.y = 0.0
        zupt_message.twist.twist.linear.z = 0.0
        zupt_message.twist.twist.angular.x = 0.0
        zupt_message.twist.twist.angular.y = 0.0
        zupt_message.twist.twist.angular.z = 0.0

        zupt_message.twist.covariance = _build_zupt_covariance(
            linear_variance_mps2=decision.linear_zupt_variance_mps2,
            angular_variance_rads2=decision.angular_zupt_variance_rads2,
        )

        self._zupt_pub.publish(zupt_message)


def _time_to_float_sec(stamp: TimeMsg) -> float:
    """Convert a ROS time stamp to floating-point seconds."""

    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _sanitize_positive_variance(variance: float, default_variance: float) -> float:
    """Clamp published variance to a finite positive scalar."""

    variance = float(variance)
    if not math.isfinite(variance) or variance <= 0.0:
        return float(default_variance)
    return variance


def _build_zupt_covariance(
    linear_variance_mps2: float, angular_variance_rads2: float
) -> list[float]:
    """Build the published stationary-twist covariance.

    The detector owns a stationary body-twist measurement, so the
    leading `3 x 3` block is an isotropic linear covariance and the
    trailing `3 x 3` block is an isotropic angular covariance.
    """

    published_linear_variance_mps2: float = _sanitize_positive_variance(
        linear_variance_mps2,
        ZuptDetectorConfig.moving_linear_variance_mps2,
    )
    published_angular_variance_rads2: float = _sanitize_positive_variance(
        angular_variance_rads2,
        ZuptDetectorConfig.moving_angular_variance_rads2,
    )
    covariance: list[float] = [0.0] * 36
    diagonal_index: int
    for diagonal_index in LINEAR_COVARIANCE_DIAGONAL_INDICES:
        covariance[diagonal_index] = published_linear_variance_mps2
    for diagonal_index in ANGULAR_COVARIANCE_DIAGONAL_INDICES:
        covariance[diagonal_index] = published_angular_variance_rads2
    return covariance
