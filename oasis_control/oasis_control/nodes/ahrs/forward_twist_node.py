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
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.timer
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    TwistWithCovarianceStamped as TwistWithCovarianceStampedMsg,
)
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg
from std_msgs.msg import String as StringMsg

from oasis_control.localization.common.frames.frame_policy import frame_matches
from oasis_control.localization.speedometer.contracts import ForwardTwistConfig
from oasis_control.localization.speedometer.contracts import ForwardTwistEstimate
from oasis_control.localization.speedometer.contracts import ZuptMeasurement
from oasis_control.localization.speedometer.forward_twist_estimator import (
    ForwardTwistEstimator,
)
from oasis_control.localization.speedometer.forward_yaw_persistence import (
    DEFAULT_MOUNT_INFO_DIRECTORY,
)
from oasis_control.localization.speedometer.forward_yaw_persistence import (
    ForwardYawPersistence,
)
from oasis_control.localization.speedometer.turn_detector import TurnDetector


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "forward_twist_node"

# ROS topics
IMU_TOPIC: str = "imu"
OUTPUT_DIAG_TOPIC: str = "forward_twist/diag"
OUTPUT_FORWARD_TWIST_TOPIC: str = "forward_twist"
ZUPT_FLAG_TOPIC: str = "zupt_flag"
ZUPT_TOPIC: str = "zupt"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_IMU_FRAME_ID: str = "imu_frame_id"
PARAM_OUTPUT_FRAME_ID: str = "output_frame_id"
PARAM_MOUNT_INFO_DIRECTORY: str = "mount_info_directory"
PARAM_STATUS_TIMER_PERIOD_SEC: str = "status_timer_period_sec"
PARAM_TURN_RATE_THRESHOLD_RADS: str = "turn_rate_threshold_rads"
PARAM_TURN_ACCEL_THRESHOLD_MPS2: str = "turn_accel_threshold_mps2"
PARAM_TURN_DIRECTION_ALIGNMENT_THRESHOLD: str = "turn_direction_alignment_threshold"
PARAM_LEARNING_ACCEL_THRESHOLD_MPS2: str = "learning_accel_threshold_mps2"
PARAM_LEARNING_MIN_SAMPLES: str = "learning_min_samples"
PARAM_LEARNING_MIN_CONFIDENCE: str = "learning_min_confidence"
PARAM_CHECKPOINT_MIN_SAMPLES: str = "checkpoint_min_samples"
PARAM_CHECKPOINT_MAX_CANDIDATE_DELTA_RAD: str = "checkpoint_max_candidate_delta_rad"
PARAM_INITIAL_FORWARD_SPEED_SIGMA_MPS: str = "initial_forward_speed_sigma_mps"
PARAM_MIN_FORWARD_SPEED_VARIANCE_MPS2: str = "min_forward_speed_variance_mps2"
PARAM_FORWARD_ACCEL_PROCESS_SIGMA_MPS2: str = "forward_accel_process_sigma_mps2"
PARAM_MAX_IMU_DT_SEC: str = "max_imu_dt_sec"

DEFAULT_BASE_FRAME_ID: str = "base_link"
DEFAULT_IMU_FRAME_ID: str = "imu_link"
DEFAULT_OUTPUT_FRAME_ID: str = "base_link"

# Timer period for publishing diagnostics when inputs are missing
#
# Units: s
#
STATUS_TIMER_PERIOD_SEC: float = 0.1

# Units: m^2/s^2
# Meaning: large placeholder variance for angular twist dimensions because this
# node does not estimate angular velocity covariance
DEFAULT_UNUSED_VARIANCE_MPS2: float = 1.0e6


################################################################################
# ROS node
################################################################################


@dataclass
class ForwardTwistDiagnosticsState:
    accepted_imu_count: int = 0
    accepted_zupt_count: int = 0
    accepted_zupt_flag_count: int = 0
    rejected_imu_count: int = 0
    rejected_zupt_count: int = 0
    has_estimate: bool = False
    has_zupt_flag: bool = False
    last_bad_imu_frame_id: str = ""
    last_status_text: str = "Waiting for IMU samples"


@dataclass
class ForwardTwistRuntimeState:
    latest_estimate: Optional[ForwardTwistEstimate] = None
    latest_zupt_flag: Optional[bool] = None


class ForwardTwistNode(rclpy.node.Node):
    """
    Skeleton ROS node for estimating forward twist from AHRS IMU and ZUPT.
    """

    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        self.declare_parameter(PARAM_IMU_FRAME_ID, DEFAULT_IMU_FRAME_ID)
        self.declare_parameter(PARAM_OUTPUT_FRAME_ID, DEFAULT_OUTPUT_FRAME_ID)
        self.declare_parameter(
            PARAM_MOUNT_INFO_DIRECTORY,
            str(DEFAULT_MOUNT_INFO_DIRECTORY),
        )
        self.declare_parameter(PARAM_STATUS_TIMER_PERIOD_SEC, STATUS_TIMER_PERIOD_SEC)
        self.declare_parameter(
            PARAM_TURN_RATE_THRESHOLD_RADS,
            ForwardTwistConfig.turn_rate_threshold_rads,
        )
        self.declare_parameter(
            PARAM_TURN_ACCEL_THRESHOLD_MPS2,
            ForwardTwistConfig.turn_accel_threshold_mps2,
        )
        self.declare_parameter(
            PARAM_TURN_DIRECTION_ALIGNMENT_THRESHOLD,
            ForwardTwistConfig.turn_direction_alignment_threshold,
        )
        self.declare_parameter(
            PARAM_LEARNING_ACCEL_THRESHOLD_MPS2,
            ForwardTwistConfig.learning_accel_threshold_mps2,
        )
        self.declare_parameter(
            PARAM_LEARNING_MIN_SAMPLES,
            ForwardTwistConfig.learning_min_samples,
        )
        self.declare_parameter(
            PARAM_LEARNING_MIN_CONFIDENCE,
            ForwardTwistConfig.learning_min_confidence,
        )
        self.declare_parameter(
            PARAM_CHECKPOINT_MIN_SAMPLES,
            ForwardTwistConfig.checkpoint_min_samples,
        )
        self.declare_parameter(
            PARAM_CHECKPOINT_MAX_CANDIDATE_DELTA_RAD,
            ForwardTwistConfig.checkpoint_max_candidate_delta_rad,
        )
        self.declare_parameter(
            PARAM_INITIAL_FORWARD_SPEED_SIGMA_MPS,
            ForwardTwistConfig.initial_forward_speed_sigma_mps,
        )
        self.declare_parameter(
            PARAM_MIN_FORWARD_SPEED_VARIANCE_MPS2,
            ForwardTwistConfig.min_forward_speed_variance_mps2,
        )
        self.declare_parameter(
            PARAM_FORWARD_ACCEL_PROCESS_SIGMA_MPS2,
            ForwardTwistConfig.forward_accel_process_sigma_mps2,
        )
        self.declare_parameter(
            PARAM_MAX_IMU_DT_SEC,
            ForwardTwistConfig.max_imu_dt_sec,
        )

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._imu_frame_id: str = str(self.get_parameter(PARAM_IMU_FRAME_ID).value)
        self._output_frame_id: str = str(
            self.get_parameter(PARAM_OUTPUT_FRAME_ID).value
        )
        self._mount_info_directory: Path = Path(
            str(self.get_parameter(PARAM_MOUNT_INFO_DIRECTORY).value)
        ).expanduser()
        self._hostname: str = socket.gethostname()
        self._status_timer_period_sec: float = float(
            self.get_parameter(PARAM_STATUS_TIMER_PERIOD_SEC).value
        )

        estimator_config: ForwardTwistConfig = ForwardTwistConfig(
            expected_imu_frame_id=self._base_frame_id,
            output_frame_id=self._output_frame_id,
            learning_accel_threshold_mps2=float(
                self.get_parameter(PARAM_LEARNING_ACCEL_THRESHOLD_MPS2).value
            ),
            learning_min_samples=int(
                self.get_parameter(PARAM_LEARNING_MIN_SAMPLES).value
            ),
            learning_min_confidence=float(
                self.get_parameter(PARAM_LEARNING_MIN_CONFIDENCE).value
            ),
            checkpoint_min_samples=int(
                self.get_parameter(PARAM_CHECKPOINT_MIN_SAMPLES).value
            ),
            checkpoint_max_candidate_delta_rad=float(
                self.get_parameter(PARAM_CHECKPOINT_MAX_CANDIDATE_DELTA_RAD).value
            ),
            initial_forward_speed_sigma_mps=float(
                self.get_parameter(PARAM_INITIAL_FORWARD_SPEED_SIGMA_MPS).value
            ),
            min_forward_speed_variance_mps2=float(
                self.get_parameter(PARAM_MIN_FORWARD_SPEED_VARIANCE_MPS2).value
            ),
            forward_accel_process_sigma_mps2=float(
                self.get_parameter(PARAM_FORWARD_ACCEL_PROCESS_SIGMA_MPS2).value
            ),
            max_imu_dt_sec=float(self.get_parameter(PARAM_MAX_IMU_DT_SEC).value),
            turn_rate_threshold_rads=float(
                self.get_parameter(PARAM_TURN_RATE_THRESHOLD_RADS).value
            ),
            turn_accel_threshold_mps2=float(
                self.get_parameter(PARAM_TURN_ACCEL_THRESHOLD_MPS2).value
            ),
            turn_direction_alignment_threshold=float(
                self.get_parameter(PARAM_TURN_DIRECTION_ALIGNMENT_THRESHOLD).value
            ),
        )

        self._diagnostics: ForwardTwistDiagnosticsState = ForwardTwistDiagnosticsState()
        self._runtime_state: ForwardTwistRuntimeState = ForwardTwistRuntimeState()
        self._turn_detector: TurnDetector = TurnDetector(
            turn_rate_threshold_rads=estimator_config.turn_rate_threshold_rads,
            turn_accel_threshold_mps2=estimator_config.turn_accel_threshold_mps2,
            turn_direction_alignment_threshold=(
                estimator_config.turn_direction_alignment_threshold
            ),
        )
        self._persistence: ForwardYawPersistence = ForwardYawPersistence(
            hostname=self._hostname,
            mount_info_directory=self._mount_info_directory,
        )
        self._estimator: ForwardTwistEstimator = ForwardTwistEstimator(
            config=estimator_config,
            persistence=self._persistence,
            turn_detector=self._turn_detector,
        )

        # ROS QoS profiles
        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        # ROS publishers
        self._diag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=StringMsg,
            topic=OUTPUT_DIAG_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._forward_twist_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=OUTPUT_FORWARD_TWIST_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        # ROS subscribers
        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=sensor_qos_profile,
        )
        self._zupt_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=ZUPT_TOPIC,
            callback=self._handle_zupt,
            qos_profile=sensor_qos_profile,
        )
        self._zupt_flag_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=BoolMsg,
            topic=ZUPT_FLAG_TOPIC,
            callback=self._handle_zupt_flag,
            qos_profile=sensor_qos_profile,
        )

        # ROS timers
        self._status_timer: rclpy.timer.Timer = self.create_timer(
            self._status_timer_period_sec,
            self._publish_status,
        )

        self._publish_status()

        self.get_logger().info("Forward twist node initialized")

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("Forward twist node deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: ImuMsg) -> None:
        if not frame_matches(message.header.frame_id, self._base_frame_id):
            self._diagnostics.rejected_imu_count += 1
            self._diagnostics.last_bad_imu_frame_id = message.header.frame_id
            self._publish_status()
            return

        self._diagnostics.accepted_imu_count += 1
        self._diagnostics.last_bad_imu_frame_id = ""

        estimate: ForwardTwistEstimate = self._estimator.update_imu(
            timestamp_ns=_time_msg_to_ns(message.header.stamp),
            orientation_xyzw=(
                float(message.orientation.x),
                float(message.orientation.y),
                float(message.orientation.z),
                float(message.orientation.w),
            ),
            angular_velocity_rads=(
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ),
            linear_acceleration_mps2=(
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ),
        )
        if estimate.imu_sample_rejected:
            self._diagnostics.rejected_imu_count += 1
        self._runtime_state.latest_estimate = estimate
        self._diagnostics.has_estimate = True

        self._publish_estimate(message, estimate)
        self._publish_status()

    def _handle_zupt(self, message: TwistWithCovarianceStampedMsg) -> None:
        self._diagnostics.accepted_zupt_count += 1

        estimate: ForwardTwistEstimate = self._estimator.update_zupt(
            measurement=ZuptMeasurement(
                timestamp_ns=_time_msg_to_ns(message.header.stamp),
                zero_velocity_variance_mps2=_sanitize_variance(
                    float(message.twist.covariance[0])
                ),
                stationary_flag=self._runtime_state.latest_zupt_flag,
            )
        )
        if estimate.zupt_sample_rejected:
            self._diagnostics.rejected_zupt_count += 1
        self._runtime_state.latest_estimate = estimate
        self._diagnostics.has_estimate = True

        self._publish_twist(
            stamp=message.header.stamp,
            estimate=estimate,
        )
        self._publish_status()

    def _handle_zupt_flag(self, message: BoolMsg) -> None:
        self._diagnostics.accepted_zupt_flag_count += 1
        self._diagnostics.has_zupt_flag = True
        self._runtime_state.latest_zupt_flag = bool(message.data)

        self._publish_status()

    def _publish_estimate(
        self,
        message: ImuMsg,
        estimate: ForwardTwistEstimate,
    ) -> None:
        self._publish_twist(
            stamp=message.header.stamp,
            estimate=estimate,
        )

    def _publish_twist(
        self,
        *,
        stamp: TimeMsg,
        estimate: ForwardTwistEstimate,
    ) -> None:
        twist_message: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
        twist_message.header.stamp = stamp
        twist_message.header.frame_id = self._output_frame_id

        forward_axis_xyz: tuple[float, float, float] = (
            estimate.forward_axis.forward_axis_xyz
        )
        twist_message.twist.twist.linear.x = (
            estimate.forward_speed_mps * forward_axis_xyz[0]
        )
        twist_message.twist.twist.linear.y = (
            estimate.forward_speed_mps * forward_axis_xyz[1]
        )
        twist_message.twist.twist.linear.z = (
            estimate.forward_speed_mps * forward_axis_xyz[2]
        )

        twist_message.twist.twist.angular.x = 0.0
        twist_message.twist.twist.angular.y = 0.0
        twist_message.twist.twist.angular.z = 0.0
        twist_message.twist.covariance = _make_placeholder_covariance(
            forward_axis_xyz=forward_axis_xyz,
            forward_speed_variance_mps2=estimate.forward_speed_variance_mps2,
        )

        self._forward_twist_pub.publish(twist_message)

    def _publish_status(self) -> None:
        status_message: StringMsg = StringMsg()
        status_message.data = self._compute_status_text()

        self._diagnostics.last_status_text = status_message.data
        self._diag_pub.publish(status_message)

    def _compute_status_text(self) -> str:
        if self._diagnostics.last_bad_imu_frame_id:
            return (
                "Received IMU frame "
                f"'{self._diagnostics.last_bad_imu_frame_id}' but expected "
                f"'{self._base_frame_id}'"
            )

        if self._diagnostics.accepted_imu_count == 0:
            return "Waiting for IMU samples"

        if self._diagnostics.accepted_zupt_count == 0:
            return "Waiting for ZUPT samples"

        if self._diagnostics.accepted_zupt_flag_count == 0:
            return "Waiting for ZUPT flag samples"

        if self._runtime_state.latest_estimate is None:
            return "Waiting for forward twist estimate"

        latest_estimate: ForwardTwistEstimate = self._runtime_state.latest_estimate
        persistence_suffix: str = ""
        if self._estimator.last_persistence_error:
            persistence_suffix = (
                f", last_persistence_error={self._estimator.last_persistence_error}"
            )
        return (
            "Forward twist running "
            f"(speed={latest_estimate.forward_speed_mps:.3f} m/s, "
            f"sigma={latest_estimate.forward_speed_sigma_mps:.3f} m/s, "
            f"candidate_yaw={latest_estimate.learning_state.candidate_forward_yaw_rad:.3f} rad, "
            f"committed_yaw={latest_estimate.learning_state.committed_forward_yaw_rad:.3f} rad, "
            f"learned={latest_estimate.forward_axis.learned}, "
            f"turn_detected={latest_estimate.turn_detected}, "
            f"learning_gated={latest_estimate.learning_state.learning_gated_by_turn}, "
            f"checkpoint_commits={latest_estimate.learning_state.checkpoint_commit_count}, "
            f"checkpoint_discards={latest_estimate.learning_state.checkpoint_discard_count}, "
            f"persistence_ok={self._estimator.persistence_success_count}, "
            f"persistence_fail={self._estimator.persistence_failure_count}, "
            f"imu_drops={self._estimator.imu_drop_count}, "
            f"zupt_drops={self._estimator.zupt_drop_count}"
            f"{persistence_suffix})"
        )


def _time_msg_to_ns(stamp: TimeMsg) -> int:
    """Convert a ROS time stamp to integer ns."""

    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _sanitize_variance(variance_mps2: float) -> float:
    """Clamp one scalar variance to a finite positive placeholder."""

    variance_value_mps2: float = float(variance_mps2)
    if not math.isfinite(variance_value_mps2) or variance_value_mps2 <= 0.0:
        return DEFAULT_UNUSED_VARIANCE_MPS2
    return variance_value_mps2


def _make_placeholder_covariance(
    *,
    forward_axis_xyz: tuple[float, float, float],
    forward_speed_variance_mps2: float,
) -> list[float]:
    """
    Build a full 6x6 placeholder covariance matrix.
    """

    covariance: list[float] = [0.0] * 36
    axis_x: float = float(forward_axis_xyz[0])
    axis_y: float = float(forward_axis_xyz[1])
    axis_z: float = float(forward_axis_xyz[2])
    scalar_variance_mps2: float = _sanitize_variance(forward_speed_variance_mps2)

    covariance[0] = scalar_variance_mps2 * axis_x * axis_x
    covariance[1] = scalar_variance_mps2 * axis_x * axis_y
    covariance[2] = scalar_variance_mps2 * axis_x * axis_z

    covariance[6] = scalar_variance_mps2 * axis_y * axis_x
    covariance[7] = scalar_variance_mps2 * axis_y * axis_y
    covariance[8] = scalar_variance_mps2 * axis_y * axis_z

    covariance[12] = scalar_variance_mps2 * axis_z * axis_x
    covariance[13] = scalar_variance_mps2 * axis_z * axis_y
    covariance[14] = scalar_variance_mps2 * axis_z * axis_z

    covariance[21] = DEFAULT_UNUSED_VARIANCE_MPS2
    covariance[28] = DEFAULT_UNUSED_VARIANCE_MPS2
    covariance[35] = DEFAULT_UNUSED_VARIANCE_MPS2
    return covariance
