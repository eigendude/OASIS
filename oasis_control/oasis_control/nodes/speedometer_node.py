################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS 2 node that estimates HUD speed from mounted IMU and ZUPT updates."""

from __future__ import annotations

import math
from typing import Iterable
from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.speedometer_core import SpeedometerConfig
from oasis_control.localization.speedometer_core import SpeedometerCore
from oasis_control.localization.speedometer_core import SpeedometerEstimate


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "speedometer"

# ROS topics
FORWARD_TWIST_TOPIC: str = "forward_twist"
IMU_TOPIC: str = "imu"
ZUPT_TOPIC: str = "zupt"
FORWARD_TWIST_FRAME_ID: str = "body_gravity"

PARAM_AXIS_LEARNING_ACCEL_THRESHOLD_MPS2: str = "axis_learning_accel_threshold_mps2"
PARAM_AXIS_LEARNING_MAX_GYRO_THRESHOLD_RADS: str = (
    "axis_learning_max_gyro_threshold_rads"
)
PARAM_AXIS_LEARNING_MIN_SAMPLES: str = "axis_learning_min_samples"
PARAM_AXIS_LEARNING_MIN_CONFIDENCE: str = "axis_learning_min_confidence"
PARAM_UNLOCKED_SPEED_STD_MPS: str = "unlocked_speed_std_mps"
PARAM_DEFAULT_FORWARD_ACCEL_STD_MPS2: str = "default_forward_accel_std_mps2"
PARAM_PROCESS_BIAS_WALK_STD_MPS2: str = "process_bias_walk_std_mps2"
PARAM_INITIAL_SPEED_STD_MPS: str = "initial_speed_std_mps"
PARAM_INITIAL_BIAS_STD_MPS2: str = "initial_bias_std_mps2"
PARAM_MAX_PREDICT_TIMESTAMP_JITTER_SEC: str = "max_predict_timestamp_jitter_sec"

DEFAULT_UNUSED_COVARIANCE: float = 1.0e6


################################################################################
# ROS node
################################################################################


class SpeedometerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        speedometer_config: SpeedometerConfig = self._read_speedometer_config()
        self._speedometer: SpeedometerCore = SpeedometerCore(speedometer_config)
        self._axis_was_learned: bool = False

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
        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_data_qos,
        )
        self._zupt_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=ZUPT_TOPIC,
            callback=self._handle_zupt,
            qos_profile=sensor_data_qos,
        )
        self.get_logger().info(
            "Speedometer node initialized "
            f"(axis accel threshold="
            f"{speedometer_config.axis_learning_accel_threshold_mps2:.3f} "
            "m/s^2)"
        )

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("Speedometer node deinitialized")

        self.destroy_node()

    def _read_speedometer_config(self) -> SpeedometerConfig:
        """
        Read ROS parameters into a ROS-agnostic speedometer config.
        """

        self.declare_parameter(
            PARAM_AXIS_LEARNING_ACCEL_THRESHOLD_MPS2,
            SpeedometerConfig.axis_learning_accel_threshold_mps2,
        )
        self.declare_parameter(
            PARAM_AXIS_LEARNING_MAX_GYRO_THRESHOLD_RADS,
            SpeedometerConfig.axis_learning_max_gyro_threshold_rads,
        )
        self.declare_parameter(
            PARAM_AXIS_LEARNING_MIN_SAMPLES,
            SpeedometerConfig.axis_learning_min_samples,
        )
        self.declare_parameter(
            PARAM_AXIS_LEARNING_MIN_CONFIDENCE,
            SpeedometerConfig.axis_learning_min_confidence,
        )
        self.declare_parameter(
            PARAM_UNLOCKED_SPEED_STD_MPS,
            SpeedometerConfig.unlocked_speed_std_mps,
        )
        self.declare_parameter(
            PARAM_DEFAULT_FORWARD_ACCEL_STD_MPS2,
            SpeedometerConfig.default_forward_accel_std_mps2,
        )
        self.declare_parameter(
            PARAM_PROCESS_BIAS_WALK_STD_MPS2,
            SpeedometerConfig.process_bias_walk_std_mps2,
        )
        self.declare_parameter(
            PARAM_INITIAL_SPEED_STD_MPS,
            SpeedometerConfig.initial_speed_std_mps,
        )
        self.declare_parameter(
            PARAM_INITIAL_BIAS_STD_MPS2,
            SpeedometerConfig.initial_bias_std_mps2,
        )
        self.declare_parameter(
            PARAM_MAX_PREDICT_TIMESTAMP_JITTER_SEC,
            SpeedometerConfig.max_predict_timestamp_jitter_sec,
        )

        return SpeedometerConfig(
            axis_learning_accel_threshold_mps2=float(
                self.get_parameter(PARAM_AXIS_LEARNING_ACCEL_THRESHOLD_MPS2).value
            ),
            axis_learning_max_gyro_threshold_rads=float(
                self.get_parameter(PARAM_AXIS_LEARNING_MAX_GYRO_THRESHOLD_RADS).value
            ),
            axis_learning_min_samples=int(
                self.get_parameter(PARAM_AXIS_LEARNING_MIN_SAMPLES).value
            ),
            axis_learning_min_confidence=float(
                self.get_parameter(PARAM_AXIS_LEARNING_MIN_CONFIDENCE).value
            ),
            unlocked_speed_std_mps=float(
                self.get_parameter(PARAM_UNLOCKED_SPEED_STD_MPS).value
            ),
            default_forward_accel_std_mps2=float(
                self.get_parameter(PARAM_DEFAULT_FORWARD_ACCEL_STD_MPS2).value
            ),
            process_bias_walk_std_mps2=float(
                self.get_parameter(PARAM_PROCESS_BIAS_WALK_STD_MPS2).value
            ),
            initial_speed_std_mps=float(
                self.get_parameter(PARAM_INITIAL_SPEED_STD_MPS).value
            ),
            initial_bias_std_mps2=float(
                self.get_parameter(PARAM_INITIAL_BIAS_STD_MPS2).value
            ),
            max_predict_timestamp_jitter_sec=float(
                self.get_parameter(PARAM_MAX_PREDICT_TIMESTAMP_JITTER_SEC).value
            ),
        )

    def _handle_imu(self, message: ImuMsg) -> None:
        """
        Update the speed estimate from one mounted base-frame IMU sample.
        """

        timestamp_sec: float = _time_to_float_sec(message.header.stamp)
        if not math.isfinite(timestamp_sec):
            self.get_logger().warning("Ignoring imu sample with non-finite timestamp")
            return

        linear_accel_covariance_mps2_2: Optional[
            tuple[tuple[float, float, float], ...]
        ] = _imu_linear_accel_covariance(message.linear_acceleration_covariance)

        estimate: Optional[SpeedometerEstimate] = self._speedometer.handle_imu_sample(
            timestamp_sec=timestamp_sec,
            linear_accel_mps2=(
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ),
            angular_velocity_rads=(
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ),
            linear_accel_covariance_mps2_2=linear_accel_covariance_mps2_2,
        )
        if estimate is None:
            return

        self._maybe_log_axis_lock(estimate)
        self._publish_estimate(message.header.stamp, estimate)

    def _handle_zupt(self, message: TwistWithCovarianceStampedMsg) -> None:
        """
        Apply a zero-velocity update and republish the HUD estimate.
        """

        timestamp_sec: float = _time_to_float_sec(message.header.stamp)
        if not self._accept_measurement_timestamp(timestamp_sec, source_name="zupt"):
            return

        measurement_variance_mps2: float = _sanitize_variance(
            message.twist.covariance[0]
        )
        estimate: Optional[SpeedometerEstimate] = self._speedometer.handle_zupt(
            timestamp_sec=timestamp_sec,
            zero_velocity_variance_mps2=measurement_variance_mps2,
        )
        if estimate is None:
            return

        self._maybe_log_axis_lock(estimate)
        self._publish_estimate(message.header.stamp, estimate)

    def _accept_measurement_timestamp(
        self, timestamp_sec: float, source_name: str
    ) -> bool:
        """
        Reject only invalid measurement timestamps.

        ZUPT messages may legitimately arrive with timestamps older than the
        latest IMU predict sample, so the node does not filter them using the
        predict timeline. The core keeps the update handling explicit.
        """

        if math.isfinite(timestamp_sec):
            return True

        self.get_logger().warning(
            f"Ignoring {source_name} sample with non-finite timestamp"
        )
        return False

    def _maybe_log_axis_lock(self, estimate: SpeedometerEstimate) -> None:
        """
        Log once when the boot-relative body motion axis locks in.
        """

        if not estimate.axis_learned or self._axis_was_learned:
            return

        self._axis_was_learned = True
        motion_axis_body: Optional[tuple[float, float, float]] = (
            estimate.motion_axis_body
        )
        if motion_axis_body is None:
            return

        self.get_logger().info(
            "Learned boot-relative body motion axis "
            f"({motion_axis_body[0]:.3f}, {motion_axis_body[1]:.3f}, "
            f"{motion_axis_body[2]:.3f}) with confidence "
            f"{estimate.axis_confidence:.3f}"
        )

    def _publish_estimate(self, stamp: TimeMsg, estimate: SpeedometerEstimate) -> None:
        """
        Publish the HUD-facing scalar speed message.

        `linear.x` is a nonnegative speed magnitude for the HUD. The associated
        variance in `covariance[0]` is the uncertainty of that scalar speed
        estimate.
        """

        message: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
        message.header.stamp = stamp
        message.header.frame_id = FORWARD_TWIST_FRAME_ID
        message.twist.twist.linear.x = max(0.0, float(estimate.speed_mps))
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0
        message.twist.twist.angular.x = 0.0
        message.twist.twist.angular.y = 0.0
        message.twist.twist.angular.z = 0.0

        covariance: list[float] = [DEFAULT_UNUSED_COVARIANCE] * 36
        covariance[0] = _sanitize_variance(estimate.speed_variance_mps2)
        message.twist.covariance = covariance
        self._forward_twist_pub.publish(message)


def _time_to_float_sec(stamp: TimeMsg) -> float:
    """Convert a ROS time stamp to floating-point seconds."""

    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _sanitize_variance(variance_mps2: float) -> float:
    """Clamp published variance to a finite positive scalar."""

    variance: float = float(variance_mps2)
    if not math.isfinite(variance) or variance <= 0.0:
        return DEFAULT_UNUSED_COVARIANCE
    return variance


def _imu_linear_accel_covariance(
    covariance: Iterable[float],
) -> Optional[tuple[tuple[float, float, float], ...]]:
    """
    Return a usable 3x3 linear-acceleration covariance matrix or None.

    Negative diagonal entries are treated as ROS-style unknown/unavailable
    covariance markers and trigger fallback to the configured default noise.
    """

    try:
        values: tuple[float, ...] = tuple(float(value) for value in covariance)
    except (TypeError, ValueError):
        return None

    if len(values) != 9:
        return None

    if not all(math.isfinite(value) for value in values):
        return None

    if values[0] < 0.0 or values[4] < 0.0 or values[8] < 0.0:
        return None

    return (
        (values[0], values[1], values[2]),
        (values[3], values[4], values[5]),
        (values[6], values[7], values[8]),
    )
