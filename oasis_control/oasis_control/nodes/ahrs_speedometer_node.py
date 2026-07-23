################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""ROS adapter for the AHRS forward-speed estimator."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
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

from oasis_control.localization.ahrs_speedometer import AhrsImuSample
from oasis_control.localization.ahrs_speedometer import AhrsSpeedometer
from oasis_control.localization.ahrs_speedometer import AhrsSpeedometerConfig
from oasis_control.localization.ahrs_speedometer import FloatArray
from oasis_control.localization.ahrs_speedometer import SpeedometerEstimate
from oasis_control.localization.ahrs_speedometer import StationaryTwistObservation
from oasis_control.ros.qos_profiles import reliable_measurement_qos
from oasis_control.ros.qos_profiles import reliable_state_qos


################################################################################
# ROS parameters
################################################################################


# Default node name
NODE_NAME: str = "ahrs_speedometer"

# ROS topics
FORWARD_TWIST_TOPIC: str = "forward_twist"
IMU_TOPIC: str = "imu"
ZUPT_FLAG_TOPIC: str = "zupt_flag"
ZUPT_TOPIC: str = "zupt"

# ROS parameters
PARAM_BASE_FRAME_ID: str = "base_frame_id"
PARAM_INITIAL_SPEED_VARIANCE_MPS2: str = "initial_speed_variance_mps2"
PARAM_FALLBACK_ACCEL_VARIANCE_MPS4: str = "fallback_accel_variance_mps4"
PARAM_FALLBACK_ANGULAR_VARIANCE_RADS2: str = "fallback_angular_variance_rads2"
PARAM_SPEED_PROCESS_NOISE_MPS2_PER_SEC: str = "speed_process_noise_mps2_per_sec"
PARAM_MAX_INTERVAL_SEC: str = "max_interval_sec"
PARAM_ZUPT_FRESHNESS_SEC: str = "zupt_freshness_sec"

DEFAULT_BASE_FRAME_ID: str = "base_link"

# Norms at or below this threshold cannot be normalized reliably
MIN_QUATERNION_NORM_SQUARED: float = 1.0e-24

# Absolute tolerance for the exact zero and isotropic ZUPT contract
ZUPT_SHAPE_ATOL: float = 1.0e-10
ZUPT_SHAPE_RTOL: float = 1.0e-8


################################################################################
# ROS node
################################################################################


class AhrsSpeedometerNode(rclpy.node.Node):
    """Publish signed ``base_link +x`` speed and all mounted angular rates.

    The supplied AHRS orientation is validated but never modified or used to
    rotate the already-mounted, gravity-free acceleration. Speed is integrated
    only after a fresh active full-twist ZUPT establishes the initial trusted
    zero state. Each stamped ZUPT is consumed at most once, and its complete
    covariance updates the speed and three angular-rate substate.
    """

    def __init__(self) -> None:
        """Initialize ROS resources and the reusable estimator."""

        super().__init__(NODE_NAME)

        # ROS parameters
        self.declare_parameter(PARAM_BASE_FRAME_ID, DEFAULT_BASE_FRAME_ID)
        defaults: AhrsSpeedometerConfig = AhrsSpeedometerConfig()
        self.declare_parameter(
            PARAM_INITIAL_SPEED_VARIANCE_MPS2,
            defaults.initial_speed_variance_mps2,
        )
        self.declare_parameter(
            PARAM_FALLBACK_ACCEL_VARIANCE_MPS4,
            defaults.fallback_accel_variance_mps4,
        )
        self.declare_parameter(
            PARAM_FALLBACK_ANGULAR_VARIANCE_RADS2,
            defaults.fallback_angular_variance_rads2,
        )
        self.declare_parameter(
            PARAM_SPEED_PROCESS_NOISE_MPS2_PER_SEC,
            defaults.speed_process_noise_mps2_per_sec,
        )
        self.declare_parameter(PARAM_MAX_INTERVAL_SEC, defaults.max_interval_sec)
        self.declare_parameter(PARAM_ZUPT_FRESHNESS_SEC, defaults.zupt_freshness_sec)

        self._base_frame_id: str = str(self.get_parameter(PARAM_BASE_FRAME_ID).value)
        self._reported_conditions: set[str] = set()
        self._accel_fallback_active: bool = False
        self._angular_fallback_active: bool = False
        self._last_zupt_flag: bool | None = None
        self._initialization_logged: bool = False
        config: AhrsSpeedometerConfig = AhrsSpeedometerConfig(
            initial_speed_variance_mps2=float(
                self.get_parameter(PARAM_INITIAL_SPEED_VARIANCE_MPS2).value
            ),
            fallback_accel_variance_mps4=float(
                self.get_parameter(PARAM_FALLBACK_ACCEL_VARIANCE_MPS4).value
            ),
            fallback_angular_variance_rads2=float(
                self.get_parameter(PARAM_FALLBACK_ANGULAR_VARIANCE_RADS2).value
            ),
            speed_process_noise_mps2_per_sec=float(
                self.get_parameter(PARAM_SPEED_PROCESS_NOISE_MPS2_PER_SEC).value
            ),
            max_interval_sec=float(self.get_parameter(PARAM_MAX_INTERVAL_SEC).value),
            zupt_freshness_sec=float(
                self.get_parameter(PARAM_ZUPT_FRESHNESS_SEC).value
            ),
        )
        self._speedometer: AhrsSpeedometer = AhrsSpeedometer(config)

        # ROS QoS profiles
        measurement_qos_profile: rclpy.qos.QoSProfile = reliable_measurement_qos()
        state_qos_profile: rclpy.qos.QoSProfile = reliable_state_qos()

        # ROS publishers
        self._forward_twist_pub: rclpy.publisher.Publisher[
            TwistWithCovarianceStampedMsg
        ] = self.create_publisher(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=FORWARD_TWIST_TOPIC,
            qos_profile=measurement_qos_profile,
        )

        # ROS subscribers
        self._imu_sub: rclpy.subscription.Subscription[ImuMsg] = (
            self.create_subscription(
                msg_type=ImuMsg,
                topic=IMU_TOPIC,
                callback=self._handle_imu,
                qos_profile=measurement_qos_profile,
            )
        )
        self._zupt_sub: rclpy.subscription.Subscription[
            TwistWithCovarianceStampedMsg
        ] = self.create_subscription(
            msg_type=TwistWithCovarianceStampedMsg,
            topic=ZUPT_TOPIC,
            callback=self._handle_zupt,
            qos_profile=measurement_qos_profile,
        )
        self._zupt_flag_sub: rclpy.subscription.Subscription[BoolMsg] = (
            self.create_subscription(
                msg_type=BoolMsg,
                topic=ZUPT_FLAG_TOPIC,
                callback=self._handle_zupt_flag,
                qos_profile=state_qos_profile,
            )
        )
        self.get_logger().info("AHRS speedometer initialized; awaiting active ZUPT")

    def stop(self) -> None:
        """Destroy the node."""

        self.get_logger().info("AHRS speedometer node deinitialized")

        self.destroy_node()

    def _handle_imu(self, message: ImuMsg) -> None:
        sample: AhrsImuSample | None = self._parse_imu(message)
        if sample is None:
            self._warn_once("invalid_imu", "Rejected invalid AHRS IMU sample")
            return
        estimate: SpeedometerEstimate | None = self._speedometer.update(sample)
        if estimate is None:
            reason: str = self._speedometer.last_rejection or "time_policy"
            self._warn_once(reason, f"Rejected AHRS IMU sample: {reason}")
            return
        if self._speedometer.last_zupt_status == "singular_innovation":
            self._error_once(
                "singular_innovation",
                "Rejected valid ZUPT: singular innovation covariance",
            )
        elif self._speedometer.last_zupt_status == "invalid_posterior":
            self._error_once(
                "invalid_posterior", "Rejected ZUPT: invalid posterior covariance"
            )
        elif self._speedometer.last_zupt_status in ("stale", "duplicate"):
            status: str = self._speedometer.last_zupt_status
            self._debug_once(
                f"zupt_{status}",
                f"ZUPT not applied: {status}",
            )
        if estimate.used_accel_covariance_fallback and not self._accel_fallback_active:
            self.get_logger().warning("Using acceleration covariance fallback")
            self._accel_fallback_active = True
        elif (
            not estimate.used_accel_covariance_fallback and self._accel_fallback_active
        ):
            self.get_logger().info("Measured acceleration covariance resumed")
            self._accel_fallback_active = False
        if (
            estimate.used_angular_covariance_fallback
            and not self._angular_fallback_active
        ):
            self.get_logger().warning("Using angular-velocity covariance fallback")
            self._angular_fallback_active = True
        elif (
            not estimate.used_angular_covariance_fallback
            and self._angular_fallback_active
        ):
            self.get_logger().info("Measured angular-velocity covariance resumed")
            self._angular_fallback_active = False
        if not estimate.initialized:
            self._debug_once(
                "uninitialized", "Accepted IMU; speedometer remains uninitialized"
            )
            return
        if not self._initialization_logged:
            self.get_logger().info(
                "AHRS speedometer initialized from ZUPT; publishing forward twist"
            )
            self._initialization_logged = True

        output: TwistWithCovarianceStampedMsg = TwistWithCovarianceStampedMsg()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._base_frame_id
        output.twist.twist.linear.x = float(estimate.mean[0])
        output.twist.twist.linear.y = 0.0
        output.twist.twist.linear.z = 0.0
        output.twist.twist.angular.x = float(estimate.mean[3])
        output.twist.twist.angular.y = float(estimate.mean[4])
        output.twist.twist.angular.z = float(estimate.mean[5])
        output.twist.covariance = estimate.covariance.reshape(36).tolist()
        self._forward_twist_pub.publish(output)

    def _handle_zupt(self, message: TwistWithCovarianceStampedMsg) -> None:
        timestamp_ns: int | None = _time_to_ns(message.header.stamp)
        mean: FloatArray = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            ],
            dtype=np.float64,
        )
        covariance: FloatArray = np.asarray(
            message.twist.covariance, dtype=np.float64
        ).reshape((6, 6))
        if timestamp_ns is None:
            self._speedometer.last_zupt_status = "invalid_input"
            self._warn_once("invalid_zupt", "Rejected invalid ZUPT")
            return
        if not is_frame_invariant_zupt(mean, covariance):
            self._speedometer.last_zupt_status = "invalid_input_covariance"
            self._warn_once(
                "invalid_zupt_covariance",
                "Rejected ZUPT: invalid input covariance",
            )
            return
        assert timestamp_ns is not None
        accepted: bool = self._speedometer.store_zupt(
            StationaryTwistObservation(
                timestamp_ns=timestamp_ns,
                mean=mean,
                covariance=covariance,
            )
        )
        if accepted:
            self.get_logger().debug("Accepted stamped ZUPT")
        else:
            self._warn_once(
                f"zupt_{self._speedometer.last_zupt_status}",
                f"Rejected ZUPT: {self._speedometer.last_zupt_status}",
            )

    def _handle_zupt_flag(self, message: BoolMsg) -> None:
        active: bool = bool(message.data)
        self._speedometer.set_zupt_active(active)
        if active != self._last_zupt_flag:
            self.get_logger().debug(f"ZUPT flag active={active}")
            self._last_zupt_flag = active

    def _parse_imu(self, message: ImuMsg) -> AhrsImuSample | None:
        if message.header.frame_id != self._base_frame_id:
            return None
        timestamp_ns: int | None = _time_to_ns(message.header.stamp)
        quaternion: FloatArray = np.array(
            [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            dtype=np.float64,
        )
        angular_velocity: FloatArray = np.array(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        linear_acceleration: FloatArray = np.array(
            [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ],
            dtype=np.float64,
        )
        if (
            timestamp_ns is None
            or not np.all(np.isfinite(quaternion))
            or not np.all(np.isfinite(angular_velocity))
            or not np.all(np.isfinite(linear_acceleration))
        ):
            return None
        quaternion_norm_squared: float = float(quaternion @ quaternion)
        if (
            not np.isfinite(quaternion_norm_squared)
            or quaternion_norm_squared <= MIN_QUATERNION_NORM_SQUARED
        ):
            return None
        normalized_quaternion: FloatArray = quaternion / np.sqrt(
            quaternion_norm_squared
        )
        if not np.all(np.isfinite(normalized_quaternion)):
            return None

        orientation_valid: bool
        orientation_covariance: FloatArray | None
        orientation_valid, orientation_covariance = _parse_imu_covariance(
            message.orientation_covariance
        )
        angular_valid: bool
        angular_covariance: FloatArray | None
        angular_valid, angular_covariance = _parse_imu_covariance(
            message.angular_velocity_covariance
        )
        acceleration_valid: bool
        acceleration_covariance: FloatArray | None
        acceleration_valid, acceleration_covariance = _parse_imu_covariance(
            message.linear_acceleration_covariance
        )
        if not orientation_valid or not angular_valid or not acceleration_valid:
            return None
        return AhrsImuSample(
            timestamp_ns=timestamp_ns,
            accel_x_mps2=float(linear_acceleration[0]),
            angular_velocity_rads=angular_velocity,
            accel_x_variance_mps4=(
                None
                if acceleration_covariance is None
                else float(acceleration_covariance[0, 0])
            ),
            angular_covariance_rads2=angular_covariance,
        )

    def _warn_once(self, key: str, message: str) -> None:
        if key not in self._reported_conditions:
            self.get_logger().warning(message)
            self._reported_conditions.add(key)

    def _error_once(self, key: str, message: str) -> None:
        if key not in self._reported_conditions:
            self.get_logger().error(message)
            self._reported_conditions.add(key)

    def _debug_once(self, key: str, message: str) -> None:
        if key not in self._reported_conditions:
            self.get_logger().debug(message)
            self._reported_conditions.add(key)


def _parse_imu_covariance(
    values: Sequence[float],
) -> tuple[bool, FloatArray | None]:
    covariance: FloatArray = np.reshape(np.asarray(values, dtype=np.float64), (3, 3))
    if covariance[0, 0] == -1.0:
        return True, None
    if not _valid_covariance(covariance):
        return False, None
    return True, covariance


def _valid_covariance(covariance: FloatArray) -> bool:
    if not np.all(np.isfinite(covariance)):
        return False
    if not np.allclose(covariance, covariance.T, rtol=1e-8, atol=1e-10):
        return False
    try:
        eigenvalues: FloatArray = np.linalg.eigvalsh(covariance)
    except np.linalg.LinAlgError:
        return False
    return bool(np.min(eigenvalues) >= -1e-10)


def is_frame_invariant_zupt(mean: FloatArray, covariance: FloatArray) -> bool:
    """Check the zero, independently isotropic ZUPT rotation contract."""

    if mean.shape != (6,) or covariance.shape != (6, 6):
        return False
    if not np.all(np.isfinite(mean)) or not np.allclose(
        mean, 0.0, rtol=0.0, atol=ZUPT_SHAPE_ATOL
    ):
        return False
    if not _valid_covariance(covariance):
        return False

    linear_variance: float = float(covariance[0, 0])
    angular_variance: float = float(covariance[3, 3])
    if linear_variance <= 0.0 or angular_variance <= 0.0:
        return False

    expected: FloatArray = np.zeros((6, 6), dtype=np.float64)
    expected[:3, :3] = np.eye(3, dtype=np.float64) * linear_variance
    expected[3:, 3:] = np.eye(3, dtype=np.float64) * angular_variance
    return bool(
        np.allclose(
            covariance,
            expected,
            rtol=ZUPT_SHAPE_RTOL,
            atol=ZUPT_SHAPE_ATOL,
        )
    )


def _time_to_ns(timestamp: TimeMsg) -> int | None:
    seconds: int = int(timestamp.sec)
    nanoseconds: int = int(timestamp.nanosec)
    if seconds < 0 or nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        return None
    return seconds * 1_000_000_000 + nanoseconds
