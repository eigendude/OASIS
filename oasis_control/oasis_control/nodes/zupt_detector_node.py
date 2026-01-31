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

from collections.abc import Sequence
from typing import Any
from typing import Optional
from typing import cast

import numpy as np
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu as ImuMsg
from std_msgs.msg import Bool as BoolMsg

from oasis_control.localization.zupt_detector import ZuptDetector
from oasis_control.localization.zupt_detector import ZuptDetectorConfig
from oasis_msgs.msg import ConductorState as ConductorStateMsg
from oasis_msgs.msg import ImuCalibration as ImuCalibrationMsg


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "zupt_detector"

# ROS topics
CONDUCTOR_STATE_TOPIC: str = "conductor_state"
IMU_CAL_TOPIC: str = "imu_calibration"
IMU_RAW_TOPIC: str = "imu_raw"
ZUPT_FLAG_TOPIC: str = "zupt_flag"
ZUPT_TOPIC: str = "zupt"

# Default base frame identifier
DEFAULT_BASE_FRAME: str = "base_link"

# (rad/s)^2 large covariance used when IMU covariance is invalid
DEFAULT_LARGE_COVARIANCE: float = 1e6

# Transition logging throttle in seconds
TRANSITION_LOG_THROTTLE_SEC: float = 1.0

# Debug logging throttle
DEBUG_LOG_THROTTLE_SEC: float = 1.0


################################################################################
# Helper functions
################################################################################


def _time_to_float_sec(stamp: TimeMsg) -> float:
    """Convert a ROS time message to seconds"""

    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _cov9_to_mat3(cov: Sequence[float]) -> np.ndarray:
    """Convert a row-major 3x3 covariance array to a matrix"""

    if len(cov) != 9:
        raise ValueError("Expected 9 covariance entries")

    cov_array: np.ndarray = np.array(cov, dtype=np.float64)
    if not np.isfinite(cov_array).all():
        raise ValueError("Covariance contains NaN or Inf")

    return cov_array.reshape((3, 3))


def _make_twist_cov(vx_var: float, *, large: float = 1e6) -> list[float]:
    """Create a TwistWithCovariance row-major 6x6 covariance array"""

    # Layout: [x y z roll pitch yaw] in row-major 6x6 order
    cov: list[float] = [large] * 36
    cov[0] = float(vx_var)
    return cov


################################################################################
# ROS node
################################################################################


class ZuptDetectorNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources
        """

        super().__init__(NODE_NAME)

        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)

        base_frame: str = str(self.get_parameter("base_frame").value)
        if not base_frame:
            self.get_logger().error("base_frame parameter is empty")
            raise RuntimeError("Missing base_frame parameter")

        self._base_frame: str = base_frame

        # ZUPT detector
        config: ZuptDetectorConfig = ZuptDetectorConfig(
            min_stationary_sec=0.1,
            min_exit_sec=0.05,
            gate_enter_chi2=50.0,
            gate_exit_chi2=80.0,
        )
        self._detector: ZuptDetector = ZuptDetector(config)

        # Cached inputs
        self._duty_cycle: float = 0.0
        self._gyro_bias: np.ndarray = np.zeros(3, dtype=np.float64)
        self._gyro_bias_cov: np.ndarray = np.zeros((3, 3), dtype=np.float64)
        self._cal_valid: bool = False
        self._last_stationary: bool = False
        self._last_transition_log_ns: int | None = None
        self._last_debug_log_ns: int | None = None

        # QoS profiles
        sensor_qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )
        conductor_qos_profile: rclpy.qos.QoSProfile = rclpy.qos.QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ROS Publishers
        self._zupt_flag_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=BoolMsg,
            topic=ZUPT_FLAG_TOPIC,
            qos_profile=sensor_qos_profile,
        )
        self._zupt_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=TwistWithCovarianceStamped,
            topic=ZUPT_TOPIC,
            qos_profile=sensor_qos_profile,
        )

        # ROS Subscribers
        self._conductor_state_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ConductorStateMsg,
                topic=CONDUCTOR_STATE_TOPIC,
                callback=self._handle_conductor_state,
                qos_profile=conductor_qos_profile,
            )
        )
        self._imu_calibration_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=ImuCalibrationMsg,
                topic=IMU_CAL_TOPIC,
                callback=self._handle_imu_calibration,
                qos_profile=sensor_qos_profile,
            )
        )
        self._imu_raw_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=ImuMsg,
            topic=IMU_RAW_TOPIC,
            callback=self._handle_imu_raw,
            qos_profile=sensor_qos_profile,
        )

        self.get_logger().info("ZUPT detector node initialized")

    def stop(self) -> None:
        self.get_logger().info("ZUPT detector node deinitialized")

        self.destroy_node()

    def _handle_conductor_state(self, message: ConductorStateMsg) -> None:
        duty_cycle: float = float(message.duty_cycle)
        self._duty_cycle = duty_cycle
        self._detector.set_duty_cycle(duty_cycle)

    def _handle_imu_calibration(self, message: ImuCalibrationMsg) -> None:
        if message.valid:
            bias: np.ndarray = np.array(
                [
                    message.gyro_bias.x,
                    message.gyro_bias.y,
                    message.gyro_bias.z,
                ],
                dtype=np.float64,
            )
            try:
                bias_cov: np.ndarray = _cov9_to_mat3(message.gyro_bias_cov)
            except ValueError as exc:
                self.get_logger().warn(
                    f"Gyro bias covariance invalid, using zeros: {exc}"
                )
                bias_cov = np.zeros((3, 3), dtype=np.float64)
            cal_valid: bool = True
        else:
            bias = np.zeros(3, dtype=np.float64)
            bias_cov = np.zeros((3, 3), dtype=np.float64)
            cal_valid = False

        self._gyro_bias = bias
        self._gyro_bias_cov = bias_cov
        self._cal_valid = cal_valid

        self._detector.set_gyro_bias(bias, bias_cov)

    def _handle_imu_raw(self, message: ImuMsg) -> None:
        omega: np.ndarray = np.array(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            dtype=np.float64,
        )

        try:
            omega_cov: np.ndarray = _cov9_to_mat3(message.angular_velocity_covariance)
        except ValueError as exc:
            self.get_logger().warn(
                f"Angular velocity covariance invalid, using large diag: {exc}"
            )
            omega_cov = np.eye(3, dtype=np.float64) * DEFAULT_LARGE_COVARIANCE

        stamp_sec: float = _time_to_float_sec(message.header.stamp)
        result: dict[str, object] = self._detector.update(
            timestamp_sec=stamp_sec,
            omega=omega,
            omega_cov=omega_cov,
        )

        stationary: bool = bool(result.get("stationary", False))
        stationary_dwell: object = result.get("stationary_dwell_sec", 0.0)
        stationary_dwell_sec: float = float(cast(float, stationary_dwell))
        diagnostics_raw: object = result.get("diagnostics", {})
        diagnostics: dict[str, Any] = cast(dict[str, Any], diagnostics_raw)
        reason: str = str(diagnostics.get("reason", "unknown"))

        # --- Throttled debug log -------------------------------------------------

        now_ns: int = int(self.get_clock().now().nanoseconds)
        throttle_ns: int = int(DEBUG_LOG_THROTTLE_SEC * 1e9)

        dt_sec: float = float(diagnostics.get("dt_sec", 0.0))
        dt_clamped: bool = bool(diagnostics.get("dt_clamped", False))
        duty_zero: bool = bool(diagnostics.get("duty_zero", False))
        omega_norm: float = float(diagnostics.get("omega_norm", 0.0))
        omega_c_norm: float = float(diagnostics.get("omega_c_norm", 0.0))
        d2: float = float(diagnostics.get("d2", 0.0))
        gate_enter: float = float(diagnostics.get("gate_enter", 0.0))
        gate_exit: float = float(diagnostics.get("gate_exit", 0.0))
        used_fallback: bool = bool(diagnostics.get("used_fallback", False))

        # Optional: expose internal detector state (super useful)
        cand_quiet: float = float(self._detector.state.candidate_quiet_time)
        cand_loud: float = float(self._detector.state.candidate_loud_time)
        dwell: float = float(self._detector.state.stationary_dwell_time)

        # Log immediately on transitions, otherwise throttle
        should_log: bool = False
        if stationary != self._last_stationary:
            should_log = True
        else:
            last_ns: int | None = self._last_debug_log_ns
            if last_ns is None or (now_ns - last_ns) >= throttle_ns:
                should_log = True

        if should_log:
            self.get_logger().debug(
                (
                    "ZUPT dbg: st=%s duty=%.3f duty0=%s dt=%.4f clamp=%s "
                    "w=%.5f wc=%.5f d2=%.3f gate=(%.3f,%.3f) fb=%s "
                    "quiet_t=%.3f loud_t=%.3f dwell=%.3f reason=%s"
                )
                % (
                    "T" if stationary else "F",
                    float(self._duty_cycle),
                    "T" if duty_zero else "F",
                    dt_sec,
                    "T" if dt_clamped else "F",
                    omega_norm,
                    omega_c_norm,
                    d2,
                    gate_enter,
                    gate_exit,
                    "T" if used_fallback else "F",
                    cand_quiet,
                    cand_loud,
                    dwell,
                    reason,
                )
            )
            self._last_debug_log_ns = now_ns

        # -------------------------------------------------------------------------

        if stationary != self._last_stationary:
            state_name: str = "enter" if stationary else "exit"
            now_ns_transition: int = int(self.get_clock().now().nanoseconds)
            throttle_ns_transition: int = int(TRANSITION_LOG_THROTTLE_SEC * 1e9)
            last_log_ns: int | None = self._last_transition_log_ns
            if (
                last_log_ns is None
                or now_ns_transition - last_log_ns >= throttle_ns_transition
            ):
                self.get_logger().debug(
                    "ZUPT %s stationary (dwell=%.3f sec, reason=%s)"
                    % (state_name, stationary_dwell_sec, reason)
                )
                self._last_transition_log_ns = now_ns_transition
            self._last_stationary = stationary

        self._zupt_flag_pub.publish(BoolMsg(data=stationary))

        zupt_vx_variance: Optional[float] = cast(
            Optional[float], result.get("zupt_vx_variance")
        )
        if zupt_vx_variance is None:
            return

        zupt_msg: TwistWithCovarianceStamped = TwistWithCovarianceStamped()
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            zupt_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            zupt_msg.header.stamp = message.header.stamp
        zupt_msg.header.frame_id = self._base_frame

        zupt_msg.twist.twist.linear.x = 0.0
        zupt_msg.twist.twist.linear.y = 0.0
        zupt_msg.twist.twist.linear.z = 0.0

        zupt_msg.twist.twist.angular.x = 0.0
        zupt_msg.twist.twist.angular.y = 0.0
        zupt_msg.twist.twist.angular.z = 0.0

        zupt_msg.twist.covariance = _make_twist_cov(
            float(zupt_vx_variance),
            large=DEFAULT_LARGE_COVARIANCE,
        )

        self._zupt_pub.publish(zupt_msg)
