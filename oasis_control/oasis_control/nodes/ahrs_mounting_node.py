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
from typing import Optional

import numpy as np
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import tf2_ros
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from geometry_msgs.msg import Vector3Stamped as Vector3StampedMsg
from sensor_msgs.msg import Imu as ImuMsg

from oasis_control.localization.imu_mount_learner import STATE_LOCKED
from oasis_control.localization.imu_mount_learner import ImuMountLearner
from oasis_control.localization.imu_mount_learner import LearnerConfig
from oasis_control.localization.imu_mount_learner import LearnerInput
from oasis_control.localization.imu_mount_learner import LearnerOutput


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_mounting"

# ROS topics
IMU_TOPIC: str = "imu"
GRAVITY_TOPIC: str = "gravity"
ACCEL_TOPIC: str = "accel"
TILT_TOPIC: str = "tilt"

# ROS parameters
PARAM_IMU_TOPIC: str = "imu_topic"
PARAM_GRAVITY_TOPIC: str = "gravity_topic"
PARAM_ACCEL_TOPIC: str = "accel_topic"
PARAM_TILT_TOPIC: str = "tilt_topic"
PARAM_BODY_FRAME_ID: str = "body_frame_id"
PARAM_IMU_FRAME_ID: str = "imu_frame_id"
PARAM_PUBLISH_RATE_HZ: str = "publish_rate_hz"
PARAM_LOCK_STDDEV_RAD: str = "lock_stddev_rad"
PARAM_CONVERGED_STDDEV_RAD: str = "converged_stddev_rad"
PARAM_LOCK_MIN_DURATION_S: str = "lock_min_duration_s"

DEFAULT_BODY_FRAME_ID: str = "base_link"
DEFAULT_IMU_FRAME_ID: str = ""
DEFAULT_PUBLISH_RATE_HZ: float = 50.0
DEFAULT_LOCK_STDDEV_RAD: float = math.radians(1.5)
DEFAULT_CONVERGED_STDDEV_RAD: float = math.radians(3.0)
DEFAULT_LOCK_MIN_DURATION_S: float = 2.0

# Units: s. Meaning: threshold for rejecting large time deltas
MAX_DT_SPIKE_S: float = 0.5


################################################################################
# ROS node
################################################################################


class AhrsMountingNode(rclpy.node.Node):
    """
    Learn IMU mounting and publish TF + gravity-referenced tilt

    TF direction convention:
        parent frame: body_frame_id
        child frame: imu_frame_id or incoming IMU header frame

        The published transform is T_BI, which rotates vectors from IMU frame
        into body frame: v_B = R_BI * v_I

    Tilt semantics:
        The published `tilt` topic is Vector3Stamped [roll, pitch, yaw] in
        radians representing body tilt relative to gravity after applying the
        learned mount. Yaw is always zero, so this is not a duplicate of full
        IMU heading
    """

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.declare_parameter(PARAM_IMU_TOPIC, IMU_TOPIC)
        self.declare_parameter(PARAM_GRAVITY_TOPIC, GRAVITY_TOPIC)
        self.declare_parameter(PARAM_ACCEL_TOPIC, ACCEL_TOPIC)
        self.declare_parameter(PARAM_TILT_TOPIC, TILT_TOPIC)
        self.declare_parameter(PARAM_BODY_FRAME_ID, DEFAULT_BODY_FRAME_ID)
        self.declare_parameter(PARAM_IMU_FRAME_ID, DEFAULT_IMU_FRAME_ID)
        self.declare_parameter(PARAM_PUBLISH_RATE_HZ, DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter(PARAM_LOCK_STDDEV_RAD, DEFAULT_LOCK_STDDEV_RAD)
        self.declare_parameter(
            PARAM_CONVERGED_STDDEV_RAD,
            DEFAULT_CONVERGED_STDDEV_RAD,
        )
        self.declare_parameter(PARAM_LOCK_MIN_DURATION_S, DEFAULT_LOCK_MIN_DURATION_S)

        self._imu_topic: str = str(self.get_parameter(PARAM_IMU_TOPIC).value)
        self._gravity_topic: str = str(self.get_parameter(PARAM_GRAVITY_TOPIC).value)
        self._accel_topic: str = str(self.get_parameter(PARAM_ACCEL_TOPIC).value)
        self._tilt_topic: str = str(self.get_parameter(PARAM_TILT_TOPIC).value)

        self._body_frame_id: str = str(self.get_parameter(PARAM_BODY_FRAME_ID).value)
        self._imu_frame_id_override: str = str(
            self.get_parameter(PARAM_IMU_FRAME_ID).value
        )
        self._publish_rate_hz: float = float(
            self.get_parameter(PARAM_PUBLISH_RATE_HZ).value
        )

        lock_stddev_rad: float = float(self.get_parameter(PARAM_LOCK_STDDEV_RAD).value)
        converged_stddev_rad: float = float(
            self.get_parameter(PARAM_CONVERGED_STDDEV_RAD).value
        )
        lock_min_duration_s: float = float(
            self.get_parameter(PARAM_LOCK_MIN_DURATION_S).value
        )

        config: LearnerConfig = LearnerConfig(
            lock_stddev_rad=lock_stddev_rad,
            converged_stddev_rad=converged_stddev_rad,
            lock_min_duration_s=lock_min_duration_s,
        )
        self._learner: ImuMountLearner = ImuMountLearner(config)

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._tilt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Vector3StampedMsg,
            topic=self._tilt_topic,
            qos_profile=qos_profile,
        )

        self._tf_broadcaster: tf2_ros.TransformBroadcaster = (
            tf2_ros.TransformBroadcaster(self)
        )
        self._tf_static_broadcaster: tf2_ros.StaticTransformBroadcaster = (
            tf2_ros.StaticTransformBroadcaster(self)
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=self._imu_topic,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )
        self._gravity_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Vector3StampedMsg,
            topic=self._gravity_topic,
            callback=self._handle_gravity,
            qos_profile=qos_profile,
        )
        self._accel_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Vector3StampedMsg,
            topic=self._accel_topic,
            callback=self._handle_accel,
            qos_profile=qos_profile,
        )

        timer_period_s: float = 0.0
        if self._publish_rate_hz > 0.0:
            timer_period_s = 1.0 / self._publish_rate_hz
        self._publish_timer = self.create_timer(timer_period_s, self._on_publish_timer)

        self._last_imu_msg: Optional[ImuMsg] = None
        self._last_imu_timestamp_s: Optional[float] = None
        self._last_gravity_vec_mps2: Optional[np.ndarray] = None
        self._last_accel_vec_mps2: Optional[np.ndarray] = None
        self._latest_output: Optional[LearnerOutput] = None

        self._last_state: Optional[str] = None
        self._static_tf_sent: bool = False
        self._last_status_log_s: Optional[float] = None

        self.get_logger().info(
            "AHRS mounting learner initialized "
            "(heading from fused IMU orientation; no separate mag topic needed)"
        )

    def stop(self) -> None:
        self.get_logger().info("AHRS mounting learner deinitialized")
        self.destroy_node()

    def _handle_gravity(self, message: Vector3StampedMsg) -> None:
        self._last_gravity_vec_mps2 = np.array(
            [message.vector.x, message.vector.y, message.vector.z],
            dtype=float,
        )

    def _handle_accel(self, message: Vector3StampedMsg) -> None:
        self._last_accel_vec_mps2 = np.array(
            [message.vector.x, message.vector.y, message.vector.z],
            dtype=float,
        )

    def _handle_imu(self, message: ImuMsg) -> None:
        timestamp_s: float = self._stamp_to_seconds(message.header.stamp)

        if self._last_imu_timestamp_s is None:
            dt_s: float = 0.0
        else:
            dt_s = max(0.0, timestamp_s - self._last_imu_timestamp_s)
        self._last_imu_timestamp_s = timestamp_s

        if dt_s > MAX_DT_SPIKE_S:
            dt_s = 0.0

        orientation_xyzw: np.ndarray = np.array(
            [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            dtype=float,
        )

        orientation_covariance: np.ndarray = self._parse_orientation_covariance(
            message.orientation_covariance,
        )

        gravity_vec_mps2: Optional[np.ndarray] = None
        if self._last_gravity_vec_mps2 is not None:
            gravity_vec_mps2 = self._last_gravity_vec_mps2.copy()
        else:
            gravity_vec_mps2 = np.array(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ],
                dtype=float,
            )

        accel_vec_mps2: Optional[np.ndarray] = None
        if self._last_accel_vec_mps2 is not None:
            accel_vec_mps2 = self._last_accel_vec_mps2.copy()
        else:
            accel_vec_mps2 = np.array(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ],
                dtype=float,
            )

        angular_velocity_rads: np.ndarray = np.array(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            dtype=float,
        )

        sample: LearnerInput = LearnerInput(
            imu_orientation_xyzw=orientation_xyzw,
            imu_orientation_cov=orientation_covariance,
            gravity_mps2=gravity_vec_mps2,
            accel_mps2=accel_vec_mps2,
            angular_velocity_rads=angular_velocity_rads,
            dt_s=dt_s,
        )

        self._latest_output = self._learner.update(sample, timestamp_s)
        self._last_imu_msg = message

        self._maybe_log_status(timestamp_s)

    def _on_publish_timer(self) -> None:
        if self._latest_output is None or self._last_imu_msg is None:
            return

        now_msg: TimeMsg = self._last_imu_msg.header.stamp

        child_imu_frame_id: str = self._resolve_imu_frame_id(self._last_imu_msg)

        transform_msg: TransformStampedMsg = TransformStampedMsg()
        transform_msg.header.stamp = now_msg
        transform_msg.header.frame_id = self._body_frame_id
        transform_msg.child_frame_id = child_imu_frame_id
        transform_msg.transform.translation.x = 0.0
        transform_msg.transform.translation.y = 0.0
        transform_msg.transform.translation.z = 0.0
        transform_msg.transform.rotation.x = float(
            self._latest_output.mount_quat_xyzw[0]
        )
        transform_msg.transform.rotation.y = float(
            self._latest_output.mount_quat_xyzw[1]
        )
        transform_msg.transform.rotation.z = float(
            self._latest_output.mount_quat_xyzw[2]
        )
        transform_msg.transform.rotation.w = float(
            self._latest_output.mount_quat_xyzw[3]
        )

        if self._latest_output.state == STATE_LOCKED:
            if not self._static_tf_sent:
                self._tf_static_broadcaster.sendTransform(transform_msg)
                self._static_tf_sent = True
        else:
            self._tf_broadcaster.sendTransform(transform_msg)
            self._static_tf_sent = False

        tilt_msg: Vector3StampedMsg = Vector3StampedMsg()
        tilt_msg.header.stamp = now_msg
        tilt_msg.header.frame_id = self._body_frame_id
        tilt_msg.vector.x = float(self._latest_output.tilt_rpy_rad[0])
        tilt_msg.vector.y = float(self._latest_output.tilt_rpy_rad[1])

        # Yaw is always zero by design to keep tilt gravity-referenced
        tilt_msg.vector.z = 0.0
        self._tilt_pub.publish(tilt_msg)

    def _resolve_imu_frame_id(self, imu_msg: ImuMsg) -> str:
        if self._imu_frame_id_override:
            return self._imu_frame_id_override
        if imu_msg.header.frame_id:
            return imu_msg.header.frame_id
        return "imu_link"

    def _parse_orientation_covariance(self, covariance: list[float]) -> np.ndarray:
        if len(covariance) != 9:
            return np.eye(3, dtype=float) * 0.25

        if float(covariance[0]) == -1.0:
            return np.eye(3, dtype=float) * 0.25

        matrix: np.ndarray = np.array(covariance, dtype=float).reshape(3, 3)

        if not np.isfinite(matrix).all():
            return np.eye(3, dtype=float) * 0.25

        # Preserve provided full covariance when valid
        matrix = 0.5 * (matrix + matrix.T)

        diag_floor: float = 1.0e-12
        matrix[0, 0] = max(float(matrix[0, 0]), diag_floor)
        matrix[1, 1] = max(float(matrix[1, 1]), diag_floor)
        matrix[2, 2] = max(float(matrix[2, 2]), diag_floor)

        return matrix

    def _maybe_log_status(self, timestamp_s: float) -> None:
        if self._latest_output is None:
            return

        state: str = self._latest_output.state
        should_log: bool = False

        if self._last_state != state:
            should_log = True
        elif self._last_status_log_s is None:
            should_log = True
        elif (timestamp_s - self._last_status_log_s) >= 2.0:
            should_log = True

        if not should_log:
            return

        self._last_state = state
        self._last_status_log_s = timestamp_s

        cov_trace: float = float(np.trace(self._latest_output.mount_cov_rad2))
        angle_std_deg: float = math.degrees(math.sqrt(max(cov_trace / 3.0, 0.0)))

        self.get_logger().info(
            f"state={state} mount_std_deg={angle_std_deg:.2f} "
            f"yaw_weight={self._latest_output.yaw_update_weight:.3f}"
        )

    def _stamp_to_seconds(self, stamp: TimeMsg) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
