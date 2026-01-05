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

import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from oasis_control.localization.madgwick_ahrs import MadgwickAhrs


################################################################################
# ROS parameters
################################################################################


NODE_NAME: str = "imu_fuser"

# ROS topics
IMU_TOPIC: str = "imu"
MAG_TOPIC: str = "magnetic_field"
IMU_FUSED_TOPIC: str = "imu_fused"

# ROS parameters
PARAM_FRAME_ID: str = "frame_id"
PARAM_PUBLISH_RATE: str = "publish_rate_hz"
PARAM_BETA: str = "beta"
PARAM_USE_MAG: str = "use_mag"
PARAM_MAG_TIMEOUT: str = "mag_timeout_s"
PARAM_MAX_DT_SPIKE: str = "max_dt_spike_s"
PARAM_STATIONARY_INIT_SAMPLES: str = "stationary_init_samples"

DEFAULT_FRAME_ID: str = ""
DEFAULT_PUBLISH_RATE_HZ: float = 0.0
DEFAULT_BETA: float = 0.1
DEFAULT_USE_MAG: bool = True
DEFAULT_MAG_TIMEOUT_S: float = 0.25
DEFAULT_MAX_DT_SPIKE_S: float = 0.2
DEFAULT_STATIONARY_INIT_SAMPLES: int = 20


################################################################################
# ROS node
################################################################################


class ImuFuserNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """

        super().__init__(NODE_NAME)

        self.declare_parameter(PARAM_FRAME_ID, DEFAULT_FRAME_ID)
        self.declare_parameter(PARAM_PUBLISH_RATE, DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter(PARAM_BETA, DEFAULT_BETA)
        self.declare_parameter(PARAM_USE_MAG, DEFAULT_USE_MAG)
        self.declare_parameter(PARAM_MAG_TIMEOUT, DEFAULT_MAG_TIMEOUT_S)
        self.declare_parameter(PARAM_MAX_DT_SPIKE, DEFAULT_MAX_DT_SPIKE_S)
        self.declare_parameter(
            PARAM_STATIONARY_INIT_SAMPLES, DEFAULT_STATIONARY_INIT_SAMPLES
        )

        self._frame_id: str = str(self.get_parameter(PARAM_FRAME_ID).value)
        self._publish_rate_hz: float = float(
            self.get_parameter(PARAM_PUBLISH_RATE).value
        )
        self._beta: float = float(self.get_parameter(PARAM_BETA).value)
        self._use_mag: bool = bool(self.get_parameter(PARAM_USE_MAG).value)
        self._mag_timeout_s: float = float(self.get_parameter(PARAM_MAG_TIMEOUT).value)
        self._max_dt_spike_s: float = float(
            self.get_parameter(PARAM_MAX_DT_SPIKE).value
        )
        self._stationary_init_samples: int = int(
            self.get_parameter(PARAM_STATIONARY_INIT_SAMPLES).value
        )

        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self._imu_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Imu,
            topic=IMU_FUSED_TOPIC,
            qos_profile=qos_profile,
        )

        self._imu_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=Imu,
            topic=IMU_TOPIC,
            callback=self._handle_imu,
            qos_profile=qos_profile,
        )
        self._mag_sub: rclpy.subscription.Subscription = self.create_subscription(
            msg_type=MagneticField,
            topic=MAG_TOPIC,
            callback=self._handle_mag,
            qos_profile=qos_profile,
        )

        self._filter: MadgwickAhrs = MadgwickAhrs(beta=self._beta)
        self._last_imu_time: Optional[float] = None
        self._last_mag_time: Optional[float] = None
        self._last_mag_message: Optional[MagneticField] = None
        self._last_dt_spike_log_time: Optional[float] = None
        self._last_mag_stale_log_time: Optional[float] = None
        self._last_publish_time: Optional[float] = None
        self._update_count: int = 0

        self.get_logger().info("IMU fuser initialized")

    def stop(self) -> None:
        self.get_logger().info("IMU fuser deinitialized")

        self.destroy_node()

    def _handle_mag(self, message: MagneticField) -> None:
        stamp: Time = message.header.stamp
        timestamp: float = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        self._last_mag_time = timestamp
        self._last_mag_message = message

    def _handle_imu(self, message: Imu) -> None:
        stamp: Time = message.header.stamp
        timestamp: float = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

        if self._last_imu_time is None:
            dt_s: float = 0.0
        else:
            dt_s = max(0.0, timestamp - self._last_imu_time)
        self._last_imu_time = timestamp

        # Protect against bag pauses, sim time jumps, and suspend/resume
        if dt_s > self._max_dt_spike_s:
            if (
                self._last_dt_spike_log_time is None
                or (timestamp - self._last_dt_spike_log_time) >= 1.0
            ):
                self.get_logger().warn(
                    f"Large IMU dt detected ({dt_s:.3f}s), skipping update"
                )
                self._last_dt_spike_log_time = timestamp
            dt_s = 0.0

        use_mag: bool = False
        mag_message: Optional[MagneticField] = None
        if self._use_mag and self._last_mag_time is not None:
            mag_age: float = timestamp - self._last_mag_time
            if mag_age <= self._mag_timeout_s:
                use_mag = True
                mag_message = self._last_mag_message
            elif (
                self._last_mag_stale_log_time is None
                or (timestamp - self._last_mag_stale_log_time) >= 1.0
            ):
                self.get_logger().warn(
                    f"Magnetometer stale by {mag_age:.3f}s, falling back to IMU"
                )
                self._last_mag_stale_log_time = timestamp
        elif self._use_mag and (
            self._last_mag_stale_log_time is None
            or (timestamp - self._last_mag_stale_log_time) >= 1.0
        ):
            self.get_logger().warn("No magnetometer data yet, using IMU only")
            self._last_mag_stale_log_time = timestamp

        gyro = message.angular_velocity
        accel = message.linear_acceleration

        if use_mag and mag_message is not None:
            mag = mag_message.magnetic_field
            self._filter.update(
                gx=gyro.x,
                gy=gyro.y,
                gz=gyro.z,
                ax=accel.x,
                ay=accel.y,
                az=accel.z,
                mx=mag.x,
                my=mag.y,
                mz=mag.z,
                dt_s=dt_s,
            )
        else:
            self._filter.update_imu(
                gx=gyro.x,
                gy=gyro.y,
                gz=gyro.z,
                ax=accel.x,
                ay=accel.y,
                az=accel.z,
                dt_s=dt_s,
            )

        self._update_count += 1

        if not self._should_publish(timestamp):
            return

        fused_message: Imu = Imu()
        fused_message.header.stamp = message.header.stamp
        fused_message.header.frame_id = (
            self._frame_id if self._frame_id else message.header.frame_id
        )

        qx, qy, qz, qw = self._filter.quaternion
        fused_message.orientation.x = qx
        fused_message.orientation.y = qy
        fused_message.orientation.z = qz
        fused_message.orientation.w = qw

        fused_message.angular_velocity = message.angular_velocity
        fused_message.angular_velocity_covariance = message.angular_velocity_covariance
        fused_message.linear_acceleration = message.linear_acceleration
        fused_message.linear_acceleration_covariance = (
            message.linear_acceleration_covariance
        )

        fused_message.orientation_covariance = self._orientation_covariance()

        self._imu_pub.publish(fused_message)

    def _should_publish(self, timestamp: float) -> bool:
        if self._publish_rate_hz <= 0.0:
            return True

        if self._last_publish_time is None:
            self._last_publish_time = timestamp
            return True

        # Units: s. Meaning: minimum time between publications
        publish_period_s: float = 1.0 / self._publish_rate_hz

        if (timestamp - self._last_publish_time) >= publish_period_s:
            self._last_publish_time = timestamp
            return True

        return False

    def _orientation_covariance(self) -> list[float]:
        if self._stationary_init_samples > 0:
            if self._update_count < self._stationary_init_samples:
                return [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Units: rad. Meaning: roll/pitch standard deviation target
        roll_pitch_std_rad: float = math.radians(5.0)

        # Units: rad. Meaning: yaw standard deviation target
        yaw_std_rad: float = math.radians(10.0)

        roll_pitch_var: float = roll_pitch_std_rad * roll_pitch_std_rad
        yaw_var: float = yaw_std_rad * yaw_std_rad

        return [
            roll_pitch_var,
            0.0,
            0.0,
            0.0,
            roll_pitch_var,
            0.0,
            0.0,
            0.0,
            yaw_var,
        ]
