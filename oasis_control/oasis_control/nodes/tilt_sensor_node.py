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

from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    AccelWithCovarianceStamped as AccelWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.tilt_pose_estimator import TiltEstimate
from oasis_control.localization.tilt_pose_estimator import TiltPoseEstimator


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "tilt_sensor"

# ROS topics
GRAVITY_TOPIC: str = "gravity"
TILT_TOPIC: str = "tilt"

# ROS parameters
PARAM_CALIBRATION_DURATION_SEC: str = "calibration_duration_sec"
DEFAULT_CALIBRATION_DURATION_SEC: float = 2.0

# sensor_msgs/Imu covariance policy for unestimated fields
UNKNOWN_COVARIANCE: list[float] = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


################################################################################
# ROS node
################################################################################


class TiltSensorNode(rclpy.node.Node):
    """
    Publish gravity-derived tilt after a stationary boot calibration window.

    The node assumes Falcon boots stationary, learns the initial gravity
    direction during startup, then publishes tilt relative to that boot
    reference. Gravity does not observe yaw, so the output orientation only
    constrains roll and pitch.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        self.declare_parameter(
            PARAM_CALIBRATION_DURATION_SEC, DEFAULT_CALIBRATION_DURATION_SEC
        )
        self._calibration_duration_sec: float = float(
            self.get_parameter(PARAM_CALIBRATION_DURATION_SEC).value
        )

        self._estimator: TiltPoseEstimator = TiltPoseEstimator(
            calibration_duration_sec=self._calibration_duration_sec
        )

        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._tilt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=ImuMsg,
            topic=TILT_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        self._gravity_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=AccelWithCovarianceStampedMsg,
            topic=GRAVITY_TOPIC,
            callback=self._handle_gravity,
            qos_profile=sensor_qos_profile,
        )

        self.get_logger().info(
            "Tilt sensor calibration started with stationary boot window of "
            f"{self._calibration_duration_sec:.1f}s"
        )

    def stop(self) -> None:
        self.get_logger().info("Tilt sensor deinitialized")

        self.destroy_node()

    def _handle_gravity(self, message: AccelWithCovarianceStampedMsg) -> None:
        timestamp_sec: float = _time_to_float_sec(message.header.stamp)
        gravity_vector_mps2: tuple[float, float, float] = (
            float(message.accel.accel.linear.x),
            float(message.accel.accel.linear.y),
            float(message.accel.accel.linear.z),
        )
        gravity_covariance_mps2_2: Optional[tuple[tuple[float, float, float], ...]] = (
            _extract_linear_covariance_mps2_2(message)
        )

        was_initialized: bool = self._estimator.initialized
        initialized_now: bool = self._estimator.add_calibration_sample(
            gravity_mps2=gravity_vector_mps2,
            timestamp_sec=timestamp_sec,
        )

        if initialized_now and not was_initialized:
            reference_gravity_unit: tuple[float, float, float] | None = (
                self._estimator.reference_gravity_unit
            )
            if reference_gravity_unit is not None:
                self.get_logger().info(
                    "Tilt sensor learned initial gravity direction "
                    f"({reference_gravity_unit[0]:.4f}, "
                    f"{reference_gravity_unit[1]:.4f}, "
                    f"{reference_gravity_unit[2]:.4f})"
                )

        if not self._estimator.initialized:
            return

        tilt_estimate: TiltEstimate | None = self._estimator.update(
            gravity_mps2=gravity_vector_mps2,
            gravity_covariance_mps2_2=gravity_covariance_mps2_2,
        )
        if tilt_estimate is None:
            return

        tilt_message: ImuMsg = ImuMsg()
        tilt_message.header = message.header

        tilt_message.orientation.x = tilt_estimate.quaternion_xyzw[0]
        tilt_message.orientation.y = tilt_estimate.quaternion_xyzw[1]
        tilt_message.orientation.z = tilt_estimate.quaternion_xyzw[2]
        tilt_message.orientation.w = tilt_estimate.quaternion_xyzw[3]
        tilt_message.orientation_covariance = tilt_estimate.orientation_covariance

        tilt_message.angular_velocity.x = 0.0
        tilt_message.angular_velocity.y = 0.0
        tilt_message.angular_velocity.z = 0.0
        tilt_message.angular_velocity_covariance = list(UNKNOWN_COVARIANCE)

        tilt_message.linear_acceleration.x = 0.0
        tilt_message.linear_acceleration.y = 0.0
        tilt_message.linear_acceleration.z = 0.0
        tilt_message.linear_acceleration_covariance = list(UNKNOWN_COVARIANCE)

        self._tilt_pub.publish(tilt_message)


def _time_to_float_sec(stamp: TimeMsg) -> float:
    """
    Convert a ROS time stamp to floating-point seconds.
    """

    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _extract_linear_covariance_mps2_2(
    message: AccelWithCovarianceStampedMsg,
) -> Optional[tuple[tuple[float, float, float], ...]]:
    """
    Extract the linear 3x3 gravity covariance block from the 6x6 ROS array.
    """

    covariance_values: tuple[float, ...] = tuple(
        float(value) for value in message.accel.covariance
    )
    if len(covariance_values) != 36:
        return None

    return (
        (
            covariance_values[0],
            covariance_values[1],
            covariance_values[2],
        ),
        (
            covariance_values[6],
            covariance_values[7],
            covariance_values[8],
        ),
        (
            covariance_values[12],
            covariance_values[13],
            covariance_values[14],
        ),
    )
