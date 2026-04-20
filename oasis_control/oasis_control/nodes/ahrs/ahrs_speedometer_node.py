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
from oasis_control.localization.speedometer.forward_twist_estimator import (
    make_flat_surface_twist_covariance,
)


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "ahrs_speedometer"

# ROS topics
IMU_TOPIC: str = "imu"
OUTPUT_DIAG_TOPIC: str = "forward_twist/diag"
OUTPUT_FORWARD_TWIST_TOPIC: str = "forward_twist"
ZUPT_FLAG_TOPIC: str = "zupt_flag"
ZUPT_TOPIC: str = "zupt"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_OUTPUT_FRAME_ID: str = "output_frame_id"
PARAM_STATUS_TIMER_PERIOD_SEC: str = "status_timer_period_sec"
PARAM_MIN_FORWARD_SPEED_VARIANCE_MPS2: str = "min_forward_speed_variance_mps2"
PARAM_MIN_FORWARD_YAW_VARIANCE_RAD2: str = "min_forward_yaw_variance_rad2"
PARAM_MIN_YAW_RATE_VARIANCE_RADS2: str = "min_yaw_rate_variance_rads2"
PARAM_FORWARD_ACCEL_PROCESS_VARIANCE_MPS2_2: str = (
    "forward_accel_process_variance_mps2_2"
)
PARAM_DEFAULT_YAW_RATE_VARIANCE_RADS2: str = "default_yaw_rate_variance_rads2"
PARAM_YAW_LEARNING_ACCEL_THRESHOLD_MPS2: str = "yaw_learning_accel_threshold_mps2"
PARAM_YAW_LEARNING_WINDOW_SIZE: str = "yaw_learning_window_size"
PARAM_YAW_LEARNING_MAX_YAW_RATE_RADS: str = "yaw_learning_max_yaw_rate_rads"
PARAM_YAW_LEARNING_MIN_SAMPLES: str = "yaw_learning_min_samples"
PARAM_YAW_LEARNING_MIN_CONFIDENCE: str = "yaw_learning_min_confidence"
PARAM_YAW_LEARNING_MAX_RESIDUAL_RATIO: str = "yaw_learning_max_residual_ratio"
PARAM_PERSISTENCE_PATH: str = "persistence_path"

DEFAULT_BASE_FRAME_ID: str = "base_link"
DEFAULT_OUTPUT_FRAME_ID: str = "base_link"

# Timer period for publishing placeholder diagnostics when inputs are missing
#
# Units: s
#
STATUS_TIMER_PERIOD_SEC: float = 0.1

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
    last_bad_imu_frame_id: str = ""
    last_status_text: str = "Waiting for IMU samples"


@dataclass
class ForwardTwistRuntimeState:
    latest_estimate: Optional[ForwardTwistEstimate] = None
    latest_zupt_flag: Optional[bool] = None


class AhrsSpeedometerNode(rclpy.node.Node):
    """
    ROS node for the AHRS speedometer pipeline.

    The published twist remains on the `ahrs/forward_twist` topic in
    `base_link` for now, but the motion model is the flat-surface
    `body_gravity` contract from the design doc rather than a free 3D
    body-axis estimate.

    Incoming ZUPT messages now carry a full 6D stationary body-twist
    measurement. This node consumes only the scalar zero-speed correction
    derived from the leading `3 x 3` linear covariance block and does not
    consume the angular stationary-twist block directly in this pass.
    """

    def __init__(self) -> None:
        """Initialize resources."""

        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        self.declare_parameter(PARAM_OUTPUT_FRAME_ID, DEFAULT_OUTPUT_FRAME_ID)
        self.declare_parameter(PARAM_STATUS_TIMER_PERIOD_SEC, STATUS_TIMER_PERIOD_SEC)
        self.declare_parameter(
            PARAM_MIN_FORWARD_SPEED_VARIANCE_MPS2,
            ForwardTwistConfig.min_forward_speed_variance_mps2,
        )
        self.declare_parameter(
            PARAM_MIN_FORWARD_YAW_VARIANCE_RAD2,
            ForwardTwistConfig.min_forward_yaw_variance_rad2,
        )
        self.declare_parameter(
            PARAM_MIN_YAW_RATE_VARIANCE_RADS2,
            ForwardTwistConfig.min_yaw_rate_variance_rads2,
        )
        self.declare_parameter(
            PARAM_FORWARD_ACCEL_PROCESS_VARIANCE_MPS2_2,
            ForwardTwistConfig.forward_accel_process_variance_mps2_2,
        )
        self.declare_parameter(
            PARAM_DEFAULT_YAW_RATE_VARIANCE_RADS2,
            ForwardTwistConfig.default_yaw_rate_variance_rads2,
        )
        self.declare_parameter(
            PARAM_YAW_LEARNING_ACCEL_THRESHOLD_MPS2,
            ForwardTwistConfig.yaw_learning_accel_threshold_mps2,
        )
        self.declare_parameter(
            PARAM_YAW_LEARNING_WINDOW_SIZE,
            ForwardTwistConfig.yaw_learning_window_size,
        )
        self.declare_parameter(
            PARAM_YAW_LEARNING_MAX_YAW_RATE_RADS,
            ForwardTwistConfig.yaw_learning_max_yaw_rate_rads,
        )
        self.declare_parameter(
            PARAM_YAW_LEARNING_MIN_SAMPLES,
            ForwardTwistConfig.yaw_learning_min_samples,
        )
        self.declare_parameter(
            PARAM_YAW_LEARNING_MIN_CONFIDENCE,
            ForwardTwistConfig.yaw_learning_min_confidence,
        )
        self.declare_parameter(
            PARAM_YAW_LEARNING_MAX_RESIDUAL_RATIO,
            ForwardTwistConfig.yaw_learning_max_residual_ratio,
        )
        self.declare_parameter(PARAM_PERSISTENCE_PATH, _default_persistence_path())

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._output_frame_id: str = str(
            self.get_parameter(PARAM_OUTPUT_FRAME_ID).value
        )
        self._status_timer_period_sec: float = float(
            self.get_parameter(PARAM_STATUS_TIMER_PERIOD_SEC).value
        )

        estimator_config: ForwardTwistConfig = ForwardTwistConfig(
            expected_imu_frame_id=self._base_frame_id,
            output_frame_id=self._output_frame_id,
            min_forward_speed_variance_mps2=float(
                self.get_parameter(PARAM_MIN_FORWARD_SPEED_VARIANCE_MPS2).value
            ),
            min_forward_yaw_variance_rad2=float(
                self.get_parameter(PARAM_MIN_FORWARD_YAW_VARIANCE_RAD2).value
            ),
            min_yaw_rate_variance_rads2=float(
                self.get_parameter(PARAM_MIN_YAW_RATE_VARIANCE_RADS2).value
            ),
            forward_accel_process_variance_mps2_2=float(
                self.get_parameter(PARAM_FORWARD_ACCEL_PROCESS_VARIANCE_MPS2_2).value
            ),
            default_yaw_rate_variance_rads2=float(
                self.get_parameter(PARAM_DEFAULT_YAW_RATE_VARIANCE_RADS2).value
            ),
            yaw_learning_accel_threshold_mps2=float(
                self.get_parameter(PARAM_YAW_LEARNING_ACCEL_THRESHOLD_MPS2).value
            ),
            yaw_learning_window_size=int(
                self.get_parameter(PARAM_YAW_LEARNING_WINDOW_SIZE).value
            ),
            yaw_learning_max_yaw_rate_rads=float(
                self.get_parameter(PARAM_YAW_LEARNING_MAX_YAW_RATE_RADS).value
            ),
            yaw_learning_min_samples=int(
                self.get_parameter(PARAM_YAW_LEARNING_MIN_SAMPLES).value
            ),
            yaw_learning_min_confidence=float(
                self.get_parameter(PARAM_YAW_LEARNING_MIN_CONFIDENCE).value
            ),
            yaw_learning_max_residual_ratio=float(
                self.get_parameter(PARAM_YAW_LEARNING_MAX_RESIDUAL_RATIO).value
            ),
            persistence_path=str(self.get_parameter(PARAM_PERSISTENCE_PATH).value),
            persistence_host=socket.gethostname(),
        )

        self._diagnostics: ForwardTwistDiagnosticsState = ForwardTwistDiagnosticsState()
        self._runtime_state: ForwardTwistRuntimeState = ForwardTwistRuntimeState()
        self._estimator: ForwardTwistEstimator = ForwardTwistEstimator(
            config=estimator_config,
        )
        self._runtime_state.latest_estimate = self._estimator.get_estimate()

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

        self.get_logger().info("AHRS speedometer node initialized")

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("AHRS speedometer node deinitialized")

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
        self._runtime_state.latest_estimate = estimate

        self._publish_estimate(message, estimate)
        self._publish_status()

    def _handle_zupt(self, message: TwistWithCovarianceStampedMsg) -> None:
        """
        Apply the scalar zero-speed part of a 6D stationary-twist update.

        The AHRS speedometer keeps a scalar forward-speed correction model, so
        it derives one linear stationary variance from the leading `3 x 3`
        covariance block and ignores the angular stationary-twist block here.
        """

        self._diagnostics.accepted_zupt_count += 1

        estimate: ForwardTwistEstimate = self._estimator.update_zupt(
            measurement=ZuptMeasurement(
                timestamp_ns=_time_msg_to_ns(message.header.stamp),
                zero_velocity_variance_mps2=(
                    _extract_stationary_linear_variance_from_zupt_covariance(
                        message.twist.covariance
                    )
                ),
                stationary_flag=self._runtime_state.latest_zupt_flag,
            )
        )
        self._runtime_state.latest_estimate = estimate

        self._publish_twist(
            stamp=message.header.stamp,
            estimate=estimate,
        )
        self._publish_status()

    def _handle_zupt_flag(self, message: BoolMsg) -> None:
        self._diagnostics.accepted_zupt_flag_count += 1
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
            math.cos(estimate.forward_yaw_rad),
            math.sin(estimate.forward_yaw_rad),
            0.0,
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
        twist_message.twist.twist.angular.z = estimate.yaw_rate_rads
        twist_message.twist.covariance = make_flat_surface_twist_covariance(
            forward_speed_mps=estimate.forward_speed_mps,
            forward_yaw_rad=estimate.forward_yaw_rad,
            forward_speed_variance_mps2=estimate.forward_speed_variance_mps2,
            forward_yaw_variance_rad2=estimate.committed_forward_yaw_variance_rad2,
            yaw_rate_variance_rads2=estimate.yaw_rate_variance_rads2,
            min_forward_speed_variance_mps2=(
                ForwardTwistConfig.min_forward_speed_variance_mps2
            ),
            min_forward_yaw_variance_rad2=(
                ForwardTwistConfig.min_forward_yaw_variance_rad2
            ),
            min_yaw_rate_variance_rads2=ForwardTwistConfig.min_yaw_rate_variance_rads2,
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
        candidate_forward_yaw_text: str = (
            "None"
            if latest_estimate.candidate_forward_yaw_rad is None
            else f"{latest_estimate.candidate_forward_yaw_rad:.3f}"
        )
        candidate_confidence_text: str = (
            "None"
            if latest_estimate.candidate_confidence is None
            else f"{latest_estimate.candidate_confidence:.3f}"
        )
        return (
            f"(committed_yaw={latest_estimate.committed_forward_yaw_rad:.3f} rad, "
            f"candidate_yaw={candidate_forward_yaw_text} rad, "
            f"forward_speed={latest_estimate.forward_speed_mps:.3f} m/s, "
            f"yaw_rate={latest_estimate.yaw_rate_rads:.3f} rad/s, "
            f"sigma_speed="
            f"{math.sqrt(latest_estimate.forward_speed_variance_mps2):.3f} m/s, "
            f"sigma_yaw="
            f"{math.sqrt(latest_estimate.committed_forward_yaw_variance_rad2):.3f} "
            f"rad, "
            f"sigma_yaw_rate="
            f"{math.sqrt(latest_estimate.yaw_rate_variance_rads2):.3f} rad/s, "
            f"candidate_confidence={candidate_confidence_text}, "
            f"accepted_learning={latest_estimate.accepted_learning_sample_count}, "
            f"discarded_learning="
            f"{latest_estimate.discarded_uncommitted_sample_count}, "
            f"checkpoint_commits={latest_estimate.checkpoint_commit_count}, "
            f"checkpoint_discards={latest_estimate.checkpoint_discard_count}, "
            f"turn_detected={latest_estimate.turn_detected}, "
            f"learning_enabled={latest_estimate.learning_enabled}, "
            f"loaded_from_persistence="
            f"{latest_estimate.loaded_from_persistence})"
        )


def _time_msg_to_ns(stamp: TimeMsg) -> int:
    """Convert a ROS time stamp to integer ns."""

    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _sanitize_variance(variance_mps2: float) -> float:
    """Clamp one scalar variance to a finite positive value."""

    variance_value_mps2: float = float(variance_mps2)
    if not math.isfinite(variance_value_mps2) or variance_value_mps2 <= 0.0:
        return ForwardTwistConfig.min_forward_speed_variance_mps2
    return variance_value_mps2


def _default_persistence_path() -> str:
    host_name: str = socket.gethostname()
    return str(
        Path("~/.ros/mount_info").expanduser() / f"ahrs_speedometer_{host_name}.yaml"
    )


def _extract_stationary_linear_variance_from_zupt_covariance(
    covariance: list[float] | tuple[float, ...],
) -> float:
    """
    Extract one scalar linear variance from a stationary-twist covariance.

    The detector publishes an isotropic leading `3 x 3` linear block by
    contract, so the mean linear variance is frame-agnostic and suitable for
    the AHRS speedometer's scalar zero-speed correction. This helper reflects
    the permanent OASIS stationary-twist restriction rather than a temporary
    implementation detail.
    """

    linear_variance_trace_mps2: float = (
        float(covariance[0]) + float(covariance[7]) + float(covariance[14])
    )
    linear_variance_mps2: float = linear_variance_trace_mps2 / 3.0
    return _sanitize_variance(linear_variance_mps2)
