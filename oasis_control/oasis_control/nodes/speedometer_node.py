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
from typing import Iterable
from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from oasis_control.localization.speed_estimator import SpeedEstimate
from oasis_control.localization.speed_estimator import SpeedEstimator
from oasis_control.localization.tilt_pose_estimator import ImuCalibration


NODE_NAME: str = "speedometer"

# ROS topics
IMU_TOPIC: str = "imu"
IMU_CALIBRATION_TOPIC: str = "imu_calibration"
MAGNETIC_FIELD_TOPIC: str = "magnetic_field"
SPEED_TOPIC: str = "speed"

# ROS parameters
PARAM_FRAME_ID: str = "frame_id"
PARAM_PUBLISH_RATE_HZ: str = "publish_rate_hz"
PARAM_GRAVITY_MPS2: str = "gravity_mps2"
PARAM_ZERO_VELOCITY_ACCEL_THRESHOLD: str = "zero_velocity_accel_threshold_mps2"
PARAM_ZERO_VELOCITY_GYRO_THRESHOLD: str = "zero_velocity_gyro_threshold_rads"
PARAM_ZERO_VELOCITY_COUNT: str = "zero_velocity_count"
PARAM_ZERO_VELOCITY_ALPHA: str = "zero_velocity_alpha"
PARAM_ZERO_VELOCITY_CLAMP: str = "zero_velocity_clamp_mps"

DEFAULT_GRAVITY_MPS2: float = 9.80665
DEFAULT_PUBLISH_RATE_HZ: float = 0.0
DEFAULT_ZERO_VELOCITY_ACCEL_THRESHOLD: float = 0.25
DEFAULT_ZERO_VELOCITY_GYRO_THRESHOLD: float = 0.05
DEFAULT_ZERO_VELOCITY_COUNT: int = 15
DEFAULT_ZERO_VELOCITY_ALPHA: float = 0.2
DEFAULT_ZERO_VELOCITY_CLAMP: float = 0.05

# Units: s. Meaning: threshold for rejecting large time deltas.
MAX_DT_SPIKE_S: float = 0.2


class SpeedometerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.declare_parameter(PARAM_FRAME_ID, "")
        self.declare_parameter(PARAM_PUBLISH_RATE_HZ, DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter(PARAM_GRAVITY_MPS2, DEFAULT_GRAVITY_MPS2)
        self.declare_parameter(
            PARAM_ZERO_VELOCITY_ACCEL_THRESHOLD,
            DEFAULT_ZERO_VELOCITY_ACCEL_THRESHOLD,
        )
        self.declare_parameter(
            PARAM_ZERO_VELOCITY_GYRO_THRESHOLD,
            DEFAULT_ZERO_VELOCITY_GYRO_THRESHOLD,
        )
        self.declare_parameter(
            PARAM_ZERO_VELOCITY_COUNT, DEFAULT_ZERO_VELOCITY_COUNT
        )
        self.declare_parameter(PARAM_ZERO_VELOCITY_ALPHA, DEFAULT_ZERO_VELOCITY_ALPHA)
        self.declare_parameter(
            PARAM_ZERO_VELOCITY_CLAMP, DEFAULT_ZERO_VELOCITY_CLAMP
        )

        self._frame_id: str = str(self.get_parameter(PARAM_FRAME_ID).value)
        publish_rate_hz: float = float(
            self.get_parameter(PARAM_PUBLISH_RATE_HZ).value
        )
        gravity_mps2: float = float(self.get_parameter(PARAM_GRAVITY_MPS2).value)
        zero_velocity_accel_threshold: float = float(
            self.get_parameter(PARAM_ZERO_VELOCITY_ACCEL_THRESHOLD).value
        )
        zero_velocity_gyro_threshold: float = float(
            self.get_parameter(PARAM_ZERO_VELOCITY_GYRO_THRESHOLD).value
        )
        zero_velocity_count: int = int(
            self.get_parameter(PARAM_ZERO_VELOCITY_COUNT).value
        )
        zero_velocity_alpha: float = float(
            self.get_parameter(PARAM_ZERO_VELOCITY_ALPHA).value
        )
        zero_velocity_clamp: float = float(
            self.get_parameter(PARAM_ZERO_VELOCITY_CLAMP).value
        )

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._estimator: SpeedEstimator = SpeedEstimator(
            gravity_mps2=gravity_mps2,
            zero_velocity_accel_threshold_mps2=zero_velocity_accel_threshold,
            zero_velocity_gyro_threshold_rads=zero_velocity_gyro_threshold,
            zero_velocity_count=zero_velocity_count,
            zero_velocity_alpha=zero_velocity_alpha,
            zero_velocity_clamp_mps=zero_velocity_clamp,
        )

        self._publish_period: float = 0.0
        if publish_rate_hz > 0.0:
            self._publish_period = 1.0 / publish_rate_hz

        self._speed_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStamped,
            topic=SPEED_TOPIC,
            qos_profile=qos_profile,
        )

        self._imu_calib_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ImuCalibrationMsg,
                topic=IMU_CALIBRATION_TOPIC,
                callback=self._handle_calibration,
                qos_profile=qos_profile,
            )
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Imu,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )

        self._magnetic_field_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=MagneticField,
                topic=MAGNETIC_FIELD_TOPIC,
                callback=self._handle_magnetic_field,
                qos_profile=qos_profile,
            )
        )

        self._calibration: Optional[ImuCalibration] = None
        self._last_imu_time: Optional[float] = None
        self._last_dt_spike_log_time: Optional[float] = None
        self._last_publish_time: Optional[float] = None
        self._last_magnetic_field: Optional[MagneticField] = None

        self.get_logger().info("Speedometer initialized")

    def stop(self) -> None:
        self.get_logger().info("Speedometer deinitialized")

        self.destroy_node()

    def _handle_calibration(self, message: ImuCalibrationMsg) -> None:
        if not message.valid:
            self.get_logger().warn("Ignoring invalid IMU calibration")
            return

        calibration: Optional[ImuCalibration] = self._calibration_from_message(
            message
        )
        if calibration is None:
            self.get_logger().warn("Invalid IMU calibration payload")
            return

        self._calibration = calibration

    def _handle_magnetic_field(self, message: MagneticField) -> None:
        self._last_magnetic_field = message

    def _handle_imu(self, message: Imu) -> None:
        if self._calibration is None:
            return

        stamp: Time = message.header.stamp
        timestamp: float = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        if self._last_imu_time is None:
            dt_s: float = 0.0
        else:
            dt_s = max(0.0, timestamp - self._last_imu_time)
        self._last_imu_time = timestamp

        if dt_s > MAX_DT_SPIKE_S:
            if (
                self._last_dt_spike_log_time is None
                or (timestamp - self._last_dt_spike_log_time) >= 1.0
            ):
                self.get_logger().warn(
                    f"Large IMU dt detected ({dt_s:.3f}s), skipping propagation"
                )
                self._last_dt_spike_log_time = timestamp
            dt_s = 0.0

        estimate: Optional[SpeedEstimate] = self._estimator.update(
            linear_accel=(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            angular_velocity=(
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            orientation_quaternion=(
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ),
            orientation_covariance=message.orientation_covariance,
            linear_accel_covariance=message.linear_acceleration_covariance,
            dt_s=dt_s,
            calibration=self._calibration,
            gravity_mps2=float(self.get_parameter(PARAM_GRAVITY_MPS2).value),
        )

        if estimate is None:
            return

        if not self._should_publish(timestamp):
            return

        message_out: TwistWithCovarianceStamped = self._twist_message(
            message, estimate
        )
        self._speed_pub.publish(message_out)

    def _should_publish(self, timestamp: float) -> bool:
        if self._publish_period <= 0.0:
            return True

        if self._last_publish_time is None:
            self._last_publish_time = timestamp
            return True

        if (timestamp - self._last_publish_time) >= self._publish_period:
            self._last_publish_time = timestamp
            return True

        return False

    def _twist_message(
        self, imu_message: Imu, estimate: SpeedEstimate
    ) -> TwistWithCovarianceStamped:
        message: TwistWithCovarianceStamped = TwistWithCovarianceStamped()
        message.header.stamp = imu_message.header.stamp
        message.header.frame_id = (
            self._frame_id if self._frame_id else imu_message.header.frame_id
        )

        message.twist.twist.linear.x = estimate.speed_mps
        message.twist.twist.linear.y = 0.0
        message.twist.twist.linear.z = 0.0

        message.twist.twist.angular = imu_message.angular_velocity

        cov: list[float] = [0.0 for _ in range(36)]
        cov[0] = estimate.variance
        cov[7] = 1.0e6
        cov[14] = 1.0e6

        angular_cov: Iterable[float] = imu_message.angular_velocity_covariance
        cov[21] = self._safe_covariance_value(angular_cov, 0)
        cov[28] = self._safe_covariance_value(angular_cov, 4)
        cov[35] = self._safe_covariance_value(angular_cov, 8)

        message.twist.covariance = cov
        return message

    def _calibration_from_message(
        self, message: ImuCalibrationMsg
    ) -> Optional[ImuCalibration]:
        accel_a: Optional[np.ndarray] = self._matrix_from_flat(
            message.accel_a, (3, 3), symmetric=False
        )
        accel_param_cov: Optional[np.ndarray] = self._matrix_from_flat(
            message.accel_param_cov, (12, 12), symmetric=True
        )
        gyro_bias_cov: Optional[np.ndarray] = self._matrix_from_flat(
            message.gyro_bias_cov, (3, 3), symmetric=True
        )

        if accel_a is None or accel_param_cov is None or gyro_bias_cov is None:
            return None

        if message.gravity_mps2 <= 0.0 or not math.isfinite(message.gravity_mps2):
            return None

        accel_bias: np.ndarray = np.array(
            [
                message.accel_bias.x,
                message.accel_bias.y,
                message.accel_bias.z,
            ],
            dtype=float,
        )

        return ImuCalibration(
            gravity_mps2=float(message.gravity_mps2),
            accel_bias_mps2=accel_bias,
            accel_a=accel_a,
            accel_param_cov=accel_param_cov,
            gyro_bias_cov=gyro_bias_cov,
        )

    def _matrix_from_flat(
        self,
        values: Iterable[float],
        shape: tuple[int, int],
        symmetric: bool,
    ) -> Optional[np.ndarray]:
        data: tuple[float, ...] = tuple(float(value) for value in values)
        if len(data) != shape[0] * shape[1]:
            return None

        if not all(math.isfinite(value) for value in data):
            return None

        matrix: np.ndarray = np.array(data, dtype=float).reshape(shape)
        if symmetric:
            return 0.5 * (matrix + matrix.T)
        return matrix

    def _safe_covariance_value(
        self, covariance: Iterable[float], index: int
    ) -> float:
        data: tuple[float, ...] = tuple(float(value) for value in covariance)
        if len(data) <= index:
            return 1.0e6

        value: float = data[index]
        if value < 0.0 or not math.isfinite(value):
            return 1.0e6

        return value
