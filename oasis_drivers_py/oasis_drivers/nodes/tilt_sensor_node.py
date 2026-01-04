################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import math
from typing import Iterable
from typing import Optional

import numpy as np
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

from oasis_drivers.localization.tilt_pose_estimator import ImuCalibration
from oasis_drivers.localization.tilt_pose_estimator import TiltPoseEstimator
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "tilt_sensor"

PARAM_IMU_TOPIC = "imu_topic"
PARAM_IMU_CALIB_TOPIC = "imu_calib_topic"
PARAM_TILT_TOPIC = "tilt_topic"
PARAM_FRAME_ID = "frame_id"
PARAM_ACCEL_TRUST_THRESHOLD = "accel_trust_threshold_mps2"
PARAM_YAW_VAR_RAD2 = "yaw_var_rad2"
PARAM_PUBLISH_TILT_RP = "publish_tilt_rp"

DEFAULT_NAMESPACE = "oasis"
DEFAULT_ROBOT_NAME = "falcon"
DEFAULT_ACCEL_TRUST_THRESHOLD_MPS2 = 1.5
DEFAULT_YAW_VAR_RAD2 = 1.0e6


################################################################################
# ROS node
################################################################################


class TiltSensorNode(rclpy.node.Node):
    def __init__(
        self,
        robot_name: str = DEFAULT_ROBOT_NAME,
        namespace: str = DEFAULT_NAMESPACE,
    ) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        imu_topic_default = f"/{namespace}/{robot_name}/imu"
        imu_calib_topic_default = f"/{namespace}/{robot_name}/imu_calibration"
        tilt_topic_default = f"/{namespace}/{robot_name}/tilt"

        self.declare_parameter(PARAM_IMU_TOPIC, imu_topic_default)
        self.declare_parameter(PARAM_IMU_CALIB_TOPIC, imu_calib_topic_default)
        self.declare_parameter(PARAM_TILT_TOPIC, tilt_topic_default)
        self.declare_parameter(PARAM_FRAME_ID, "")
        self.declare_parameter(
            PARAM_ACCEL_TRUST_THRESHOLD, DEFAULT_ACCEL_TRUST_THRESHOLD_MPS2
        )
        self.declare_parameter(PARAM_YAW_VAR_RAD2, DEFAULT_YAW_VAR_RAD2)
        self.declare_parameter(PARAM_PUBLISH_TILT_RP, False)

        imu_topic = str(self.get_parameter(PARAM_IMU_TOPIC).value)
        imu_calib_topic = str(self.get_parameter(PARAM_IMU_CALIB_TOPIC).value)
        tilt_topic = str(self.get_parameter(PARAM_TILT_TOPIC).value)

        self._frame_id = str(self.get_parameter(PARAM_FRAME_ID).value)
        accel_trust_threshold = float(
            self.get_parameter(PARAM_ACCEL_TRUST_THRESHOLD).value
        )
        yaw_variance = float(self.get_parameter(PARAM_YAW_VAR_RAD2).value)
        publish_tilt_rp = bool(self.get_parameter(PARAM_PUBLISH_TILT_RP).value)

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._estimator = TiltPoseEstimator(
            accel_trust_threshold_mps2=accel_trust_threshold,
            yaw_variance_rad2=yaw_variance,
        )

        self._tilt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Imu,
            topic=tilt_topic,
            qos_profile=qos_profile,
        )

        self._tilt_rp_pub: Optional[rclpy.publisher.Publisher] = None
        if publish_tilt_rp:
            self._tilt_rp_pub = self.create_publisher(
                msg_type=Vector3Stamped,
                topic=f"{tilt_topic}_rp",
                qos_profile=qos_profile,
            )

        self._imu_calib_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ImuCalibrationMsg,
                topic=imu_calib_topic,
                callback=self._handle_calibration,
                qos_profile=qos_profile,
            )
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Imu,
            topic=imu_topic,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )

        self._calibration: Optional[ImuCalibration] = None
        self._last_imu_time: Optional[float] = None

        self.get_logger().info("Tilt sensor initialized")

    def stop(self) -> None:
        self.get_logger().info("Tilt sensor deinitialized")

        self.destroy_node()

    def _handle_calibration(self, message: ImuCalibrationMsg) -> None:
        if not message.valid:
            self.get_logger().warn("Ignoring invalid IMU calibration")
            return

        calibration = self._calibration_from_message(message)
        if calibration is None:
            self.get_logger().warn("Invalid IMU calibration payload")
            return

        self._calibration = calibration
        self.get_logger().info("IMU calibration updated")

    def _handle_imu(self, message: Imu) -> None:
        if self._calibration is None:
            return

        stamp = message.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if self._last_imu_time is None:
            dt_s = 0.0
        else:
            dt_s = max(0.0, timestamp - self._last_imu_time)
        self._last_imu_time = timestamp

        updated = self._estimator.update(
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
            dt_s=dt_s,
            linear_accel_covariance=message.linear_acceleration_covariance,
            angular_velocity_covariance=message.angular_velocity_covariance,
            calibration=self._calibration,
        )

        if not updated:
            return

        tilt_message = Imu()
        tilt_message.header.stamp = message.header.stamp
        tilt_message.header.frame_id = (
            self._frame_id if self._frame_id else message.header.frame_id
        )

        orientation = self._estimator.orientation_quaternion()
        tilt_message.orientation.x = orientation[0]
        tilt_message.orientation.y = orientation[1]
        tilt_message.orientation.z = orientation[2]
        tilt_message.orientation.w = orientation[3]
        tilt_message.orientation_covariance = (
            self._estimator.orientation_covariance_rpy()
        )

        tilt_message.angular_velocity = message.angular_velocity
        tilt_message.angular_velocity_covariance = message.angular_velocity_covariance

        tilt_message.linear_acceleration = message.linear_acceleration
        tilt_message.linear_acceleration_covariance = (
            message.linear_acceleration_covariance
        )

        self._tilt_pub.publish(tilt_message)

        if self._tilt_rp_pub is not None:
            roll, pitch, _ = self._estimator.attitude_rpy()
            rp_message = Vector3Stamped()
            rp_message.header.stamp = message.header.stamp
            rp_message.header.frame_id = tilt_message.header.frame_id
            rp_message.vector.x = roll
            rp_message.vector.y = pitch
            rp_message.vector.z = 0.0
            self._tilt_rp_pub.publish(rp_message)

    def _calibration_from_message(
        self, message: ImuCalibrationMsg
    ) -> Optional[ImuCalibration]:
        accel_a = self._matrix_from_flat(message.accel_a, (3, 3), symmetric=False)
        accel_param_cov = self._matrix_from_flat(
            message.accel_param_cov, (12, 12), symmetric=True
        )
        gyro_bias_cov = self._matrix_from_flat(
            message.gyro_bias_cov, (3, 3), symmetric=True
        )

        if accel_a is None or accel_param_cov is None or gyro_bias_cov is None:
            return None

        if message.gravity_mps2 <= 0.0 or not math.isfinite(message.gravity_mps2):
            return None

        accel_bias = np.array(
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
        data = tuple(float(value) for value in values)
        if len(data) != shape[0] * shape[1]:
            return None

        if not all(math.isfinite(value) for value in data):
            return None

        matrix = np.array(data, dtype=float).reshape(shape)
        if symmetric:
            return 0.5 * (matrix + matrix.T)
        return matrix
