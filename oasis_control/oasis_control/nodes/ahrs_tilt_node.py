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
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from rclpy.time import Time
from sensor_msgs.msg import Imu as ImuMsg
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from oasis_control.localization.ahrs_tilt_estimator import AhrsTiltEstimate
from oasis_control.localization.ahrs_tilt_estimator import AhrsTiltEstimator
from oasis_control.localization.common.algebra.covariance import (
    UNKNOWN_ORIENTATION_COVARIANCE,
)
from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.frames.mounting import MountedGravitySample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import apply_mounting_to_gravity
from oasis_control.localization.common.frames.mounting import make_mounting_transform
from oasis_control.localization.common.validation.gravity_validation import (
    GravityValidationResult,
)
from oasis_control.localization.common.validation.gravity_validation import (
    validate_gravity_sample,
)


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_tilt"

# ROS topics
GRAVITY_TOPIC: str = "gravity"
IMU_TOPIC: str = "imu"
TILT_TOPIC: str = "tilt"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_IMU_FRAME_ID: str = "imu_frame_id"

DEFAULT_BASE_FRAME_ID: str = "base_link"
DEFAULT_IMU_FRAME_ID: str = "imu_link"

# sensor_msgs/Imu covariance policy for unestimated fields
UNKNOWN_COVARIANCE: list[float] = list(UNKNOWN_ORIENTATION_COVARIANCE)

# Units: s
# Meaning: newest gravity sample age accepted when pairing covariance to an
# AHRS-orientation tilt mean
MAX_GRAVITY_COVARIANCE_AGE_SEC: float = 0.2


################################################################################
# ROS node
################################################################################


class AhrsTiltNode(rclpy.node.Node):
    """
    Publish an AHRS tilt mean with gravity-derived covariance.

    The input `imu` stream is expected in `base_link` and provides the tilt
    mean. The input `gravity` stream is expected in `imu_link`, rotated through
    the established IMU-to-base mounting transform, and used only for the
    roll/pitch covariance of the published `tilt` output.
    """

    def __init__(self, *, tf_buffer: Optional[Buffer] = None) -> None:
        """
        Initialize resources.
        """

        super().__init__(NODE_NAME)

        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        self.declare_parameter(PARAM_IMU_FRAME_ID, DEFAULT_IMU_FRAME_ID)

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._imu_frame_id: str = str(self.get_parameter(PARAM_IMU_FRAME_ID).value)
        self._estimator: AhrsTiltEstimator = AhrsTiltEstimator()
        self._latest_gravity_sample: Optional[GravitySample] = None
        self._mounting_transform: Optional[MountingTransform] = None

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
        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_qos_profile,
        )

        self._tf_buffer: Buffer = tf_buffer if tf_buffer is not None else Buffer()
        self._transform_listener: Optional[TransformListener] = None
        if tf_buffer is None:
            self._transform_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info("AHRS tilt node initialized")

    def stop(self) -> None:
        self.get_logger().info("AHRS tilt node deinitialized")

        self.destroy_node()

    def _handle_gravity(self, message: AccelWithCovarianceStampedMsg) -> None:
        validation_result: GravityValidationResult = validate_gravity_sample(
            timestamp_ns=_time_msg_to_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            expected_frame_id=self._imu_frame_id,
            gravity_mps2=(
                float(message.accel.accel.linear.x),
                float(message.accel.accel.linear.y),
                float(message.accel.accel.linear.z),
            ),
            gravity_covariance_row_major=tuple(
                float(value) for value in message.accel.covariance
            ),
        )
        if not validation_result.accepted or validation_result.sample is None:
            if validation_result.rejection_reason == "bad_frame":
                self.get_logger().warning(
                    "Ignoring gravity sample with unexpected frame_id "
                    f"'{message.header.frame_id}', expected "
                    f"'{self._imu_frame_id}'"
                )
            return

        self._latest_gravity_sample = validation_result.sample

    def _handle_imu(self, message: ImuMsg) -> None:
        if message.header.frame_id != self._base_frame_id:
            self.get_logger().warning(
                "Ignoring AHRS tilt sample with unexpected frame_id "
                f"'{message.header.frame_id}', expected '{self._base_frame_id}'"
            )
            return

        mounted_gravity_sample: Optional[MountedGravitySample] = (
            self._resolve_fresh_gravity_sample(
                imu_timestamp_ns=_time_msg_to_ns(message.header.stamp)
            )
        )

        tilt_estimate: AhrsTiltEstimate | None = self._estimator.update(
            orientation_xyzw=(
                float(message.orientation.x),
                float(message.orientation.y),
                float(message.orientation.z),
                float(message.orientation.w),
            ),
            gravity_mps2=(
                mounted_gravity_sample.gravity_mps2
                if mounted_gravity_sample is not None
                else None
            ),
            gravity_covariance_mps2_2=(
                mounted_gravity_sample.gravity_covariance_mps2_2
                if mounted_gravity_sample is not None
                else None
            ),
        )
        if tilt_estimate is None:
            return

        tilt_message: ImuMsg = ImuMsg()
        tilt_message.header = message.header
        tilt_message.header.frame_id = self._base_frame_id

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

    def _resolve_fresh_gravity_sample(
        self, *, imu_timestamp_ns: int
    ) -> Optional[MountedGravitySample]:
        if self._latest_gravity_sample is None:
            return None

        gravity_age_ns: int = (
            imu_timestamp_ns - self._latest_gravity_sample.timestamp_ns
        )
        max_gravity_age_ns: int = int(MAX_GRAVITY_COVARIANCE_AGE_SEC * 1.0e9)

        if gravity_age_ns < 0 or gravity_age_ns > max_gravity_age_ns:
            return None

        mounting_transform: Optional[MountingTransform] = self._resolve_mounting()
        if mounting_transform is None:
            return None

        return apply_mounting_to_gravity(
            self._latest_gravity_sample,
            mounting_transform,
        )

    def _resolve_mounting(self) -> Optional[MountingTransform]:
        if self._mounting_transform is not None:
            return self._mounting_transform

        try:
            transform_message: TransformStampedMsg = self._tf_buffer.lookup_transform(
                self._base_frame_id,
                self._imu_frame_id,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warning(
                "Unable to resolve gravity mounting for AHRS tilt covariance: "
                f"{error}"
            )
            return None

        quaternion_xyzw: Optional[tuple[float, float, float, float]] = (
            normalize_quaternion_xyzw(
                (
                    float(transform_message.transform.rotation.x),
                    float(transform_message.transform.rotation.y),
                    float(transform_message.transform.rotation.z),
                    float(transform_message.transform.rotation.w),
                )
            )
        )
        if quaternion_xyzw is None:
            self.get_logger().warning(
                "Ignoring invalid mounting quaternion while rotating gravity "
                "covariance for AHRS tilt"
            )
            return None

        self._mounting_transform = make_mounting_transform(
            parent_frame_id=self._base_frame_id,
            child_frame_id=self._imu_frame_id,
            quaternion_xyzw=quaternion_xyzw,
        )
        return self._mounting_transform


def _time_msg_to_ns(stamp: TimeMsg) -> int:
    """
    Convert a ROS time stamp into integer nanoseconds.
    """

    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
