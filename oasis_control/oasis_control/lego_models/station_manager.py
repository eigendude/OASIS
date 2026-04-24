################################################################################
#
#  Copyright (C) 2022-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a LEGO train station's microcontroller
#

import math
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.task
from builtin_interfaces.msg import Time as TimeMsg

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_drivers.telemetrix.telemetrix_types import DigitalMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import DigitalReading as DigitalReadingMsg
from oasis_msgs.msg import DigitalWriteCommand as DigitalWriteCommandMsg
from oasis_msgs.msg import PWMWriteCommand as PWMWriteCommandMsg
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetDigitalMode as SetDigitalModeSvc


################################################################################
# Hardware configuration
################################################################################


# External AREF is tied to a voltage regulator for stable ADC scaling
#
# Observed values:
#
#   5.02 V on 2025-12-18
#   4.95 V on 2026-01-03
#   4.98 V on 2026-01-04
#
# Let's just use 5 V, which is within 1% of all observed values. The regulator
# voltage is relatively stable compared to the 12 V train load.
#
AREF_VOLTAGE: float = 5.00  # Volts
VIN_GAIN: float = 0.9887  # Empirical system calibration to match DMM at ~12V

# Voltage dividers
# R1 is the input-side resistor, R2 is the ground-side resistor
VSS_R1: float = 26.62  # KΩ
VSS_R2: float = 9.83  # KΩ

# Motor wire A (blue wire) voltage divider
MOTOR_VOLTAGE_A_R1: float = 17.80  # KΩ
MOTOR_VOLTAGE_A_R2: float = 9.84  # KΩ
# R3: 1KΩ resistor to A3 pin, 10nF cap to GND

# Motor wire B (green wire) voltage divider
MOTOR_VOLTAGE_B_R1: float = 17.78  # KΩ
MOTOR_VOLTAGE_B_R2: float = 9.89  # KΩ
# R3: 1KΩ resistor to A4 pin, 10nF cap to GND


################################################################################
# Filtering parameters
################################################################################


# Higher beta adapts faster to changing noise when motors toggle
STDDEV_BETA: float = 0.2

# Reject motor voltage updates when the A/B sample pair is too stale to
# represent the same H-bridge state
MAX_MOTOR_VOLTAGE_SKEW_SECS: float = 0.15


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"
SUBSCRIBE_DIGITAL_READING = "digital_reading"

# Service clients
CLIENT_SET_ANALOG_MODE = "set_analog_mode"
CLIENT_SET_DIGITAL_MODE = "set_digital_mode"

# Command publishers
PUBLISH_MOTOR_DIR_CMD = "digital_write_cmd"
PUBLISH_MOTOR_PWM_CMD = "pwm_write_cmd"


################################################################################
# Manager
################################################################################


class StationManager:
    """
    A ROS node that manages a LEGO train power conductor's conductor.

    Motor voltage is measured as the signed H-bridge differential output,
    computed from the reconstructed wire-A and wire-B voltages. The reported
    motor voltage stddev combines two effects: an EWMA-based base variance from
    the measured differential stream, and an extra uncertainty term for
    asynchronous A/B ADC sampling. That timing-skew term inflates the stddev
    when the two channels were sampled at different times while their local
    slopes indicate the H-bridge output was changing.
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        vss_pin: int,
        motor_pwm_pin: int,
        motor_dir_pin: int,
        motor_ff1_pin: int,
        motor_ff2_pin: int,
        motor_current_pin: int,
        motor_voltage_a_pin: int,
        motor_voltage_b_pin: int,
    ) -> None:
        """
        Initialize resources.
        """
        # Construction parameters
        self._node = node
        self._vss_pin: int = vss_pin
        self._motor_pwm_pin: int = motor_pwm_pin
        self._motor_dir_pin: int = motor_dir_pin
        self._motor_ff1_pin: int = motor_ff1_pin
        self._motor_ff2_pin: int = motor_ff2_pin
        self._motor_current_pin: int = motor_current_pin
        self._motor_voltage_a_pin: int = motor_voltage_a_pin
        self._motor_voltage_b_pin: int = motor_voltage_b_pin

        # Initialize hardware state
        self._supply_voltage: float = 0.0
        self._supply_voltage_mu: float = 0.0
        self._supply_voltage_var: float = 0.0
        self._supply_voltage_initialized: bool = False
        self._supply_voltage_stddev: float = 0.0
        self._motor_voltage_a: float = 0.0
        self._motor_voltage_a_initialized: bool = False
        self._motor_voltage_a_timestamp_sec: Optional[float] = None
        self._prev_motor_voltage_a: Optional[float] = None
        self._prev_motor_voltage_a_timestamp_sec: Optional[float] = None
        self._motor_voltage_b: float = 0.0
        self._motor_voltage_b_initialized: bool = False
        self._motor_voltage_b_timestamp_sec: Optional[float] = None
        self._prev_motor_voltage_b: Optional[float] = None
        self._prev_motor_voltage_b_timestamp_sec: Optional[float] = None
        self._motor_voltage: float = 0.0
        self._motor_voltage_mu: float = 0.0
        self._motor_voltage_base_var: float = 0.0
        self._motor_voltage_initialized: bool = False
        self._motor_voltage_stddev: float = 0.0
        self._motor_duty_cycle: float = 0.0
        self._motor_current: float = 0.0
        self._motor_ff1_state: bool = False
        self._motor_ff1_count: int = 0
        self._motor_ff2_state: bool = False
        self._motor_ff2_count: int = 0

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        cmd_qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
        )
        self._motor_dir_cmd_pub: rclpy.publisher.Publisher = (
            self._node.create_publisher(
                msg_type=DigitalWriteCommandMsg,
                topic=PUBLISH_MOTOR_DIR_CMD,
                qos_profile=cmd_qos,
            )
        )
        self._motor_pwm_cmd_pub: rclpy.publisher.Publisher = (
            self._node.create_publisher(
                msg_type=PWMWriteCommandMsg,
                topic=PUBLISH_MOTOR_PWM_CMD,
                qos_profile=cmd_qos,
            )
        )

        # Subscribers
        self._analog_reading_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=AnalogReadingMsg,
                topic=SUBSCRIBE_ANALOG_READING,
                callback=self._on_analog_reading,
                qos_profile=qos_profile,
            )
        )
        self._digital_reading_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=DigitalReadingMsg,
                topic=SUBSCRIBE_DIGITAL_READING,
                callback=self._on_digital_reading,
                qos_profile=qos_profile,
            )
        )

        # Service clients
        self._set_analog_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetAnalogModeSvc, srv_name=CLIENT_SET_ANALOG_MODE
        )
        self._set_digital_mode_client: rclpy.client.Client = self._node.create_client(
            srv_type=SetDigitalModeSvc, srv_name=CLIENT_SET_DIGITAL_MODE
        )

    @property
    def supply_voltage(self) -> float:
        return self._supply_voltage

    @property
    def supply_voltage_stddev(self) -> float:
        return self._supply_voltage_stddev

    @property
    def motor_voltage(self) -> float:
        return self._motor_voltage

    @property
    def motor_voltage_stddev(self) -> float:
        return self._motor_voltage_stddev

    @property
    def motor_duty_cycle(self) -> float:
        return self._motor_duty_cycle

    @property
    def motor_current(self) -> float:
        return self._motor_current

    @property
    def motor_ff1_count(self) -> int:
        return self._motor_ff1_count

    @property
    def motor_ff2_count(self) -> int:
        return self._motor_ff2_count

    def initialize(self) -> bool:
        self._node.get_logger().debug("Waiting for station services")
        self._node.get_logger().debug("  - Waiting for set_analog_mode...")
        self._set_analog_mode_client.wait_for_service()
        self._node.get_logger().debug("  - Waiting for set_digital_mode...")
        self._set_digital_mode_client.wait_for_service()

        self._node.get_logger().debug("Starting station configuration")

        # Voltage supply source (VSS)
        self._node.get_logger().debug(f"Enabling VSS on A{self._vss_pin}")
        if not self._set_analog_mode(self._vss_pin, AnalogMode.INPUT):
            return False

        #
        # Pololu md07a motor driver 18v15
        #

        # Motor PWM
        self._node.get_logger().debug(f"Enabling motor PWM on D{self._motor_pwm_pin}")
        if not self._set_digital_mode(self._motor_pwm_pin, DigitalMode.PWM):
            return False

        # Motor DIR
        self._node.get_logger().debug(f"Enabling motor DIR on D{self._motor_dir_pin}")
        if not self._set_digital_mode(self._motor_dir_pin, DigitalMode.OUTPUT):
            return False

        # Motor FF1
        self._node.get_logger().debug(f"Enabling motor FF1 on D{self._motor_ff1_pin}")
        if not self._set_digital_mode(self._motor_ff1_pin, DigitalMode.INPUT):
            return False

        # Motor FF2
        self._node.get_logger().debug(f"Enabling motor FF2 on D{self._motor_ff2_pin}")
        if not self._set_digital_mode(self._motor_ff2_pin, DigitalMode.INPUT):
            return False

        #
        # Sparkfun ACS712 current sensor
        #

        # Output voltage (VO)
        self._node.get_logger().debug(
            f"Enabling current sensor VO on A{self._motor_current_pin}"
        )
        if not self._set_analog_mode(self._motor_current_pin, AnalogMode.INPUT):
            return False

        # H-bridge output A voltage
        self._node.get_logger().debug(
            f"Enabling motor voltage A on A{self._motor_voltage_a_pin}"
        )
        if not self._set_analog_mode(self._motor_voltage_a_pin, AnalogMode.INPUT):
            return False

        # H-bridge output B voltage
        self._node.get_logger().debug(
            f"Enabling motor voltage B on A{self._motor_voltage_b_pin}"
        )
        if not self._set_analog_mode(self._motor_voltage_b_pin, AnalogMode.INPUT):
            return False

        self._node.get_logger().info("Station manager initialized successfully")

        return True

    def set_motor_direction(self, reverse: bool) -> None:
        """Publish command for motor direction"""
        dir_cmd = DigitalWriteCommandMsg()
        dir_cmd.digital_pin = self._motor_dir_pin
        dir_cmd.digital_value = reverse

        self._motor_dir_cmd_pub.publish(dir_cmd)

    def set_motor_pwm(self, target_magnitude: float, reverse: bool) -> None:
        """Publish command for motor PWM"""
        pwm_cmd = PWMWriteCommandMsg()
        pwm_cmd.digital_pin = self._motor_pwm_pin
        pwm_cmd.duty_cycle = target_magnitude

        self._motor_pwm_cmd_pub.publish(pwm_cmd)

        # Update state
        self._motor_duty_cycle = -target_magnitude if reverse else target_magnitude

    def _decode_divider_voltage(
        self,
        analog_voltage: float,
        r1_kohm: float,
        r2_kohm: float,
    ) -> float:
        # Reconstruct the source-side voltage from the divider tap voltage
        return analog_voltage * (r1_kohm + r2_kohm) / r2_kohm

    def _time_msg_to_sec(self, stamp: TimeMsg) -> float:
        sec: int = int(stamp.sec)
        nanosec: int = int(stamp.nanosec)
        return float(sec) + float(nanosec) * 1.0e-9

    def _estimate_slope(
        self,
        current_value: float,
        current_timestamp_sec: Optional[float],
        previous_value: Optional[float],
        previous_timestamp_sec: Optional[float],
    ) -> Optional[float]:
        if (
            current_timestamp_sec is None
            or previous_value is None
            or previous_timestamp_sec is None
        ):
            return None

        delta_time_sec: float = current_timestamp_sec - previous_timestamp_sec
        if delta_time_sec <= 0.0:
            return None

        return (current_value - previous_value) / delta_time_sec

    def _update_motor_voltage(self) -> None:
        if (
            not self._motor_voltage_a_initialized
            or not self._motor_voltage_b_initialized
        ):
            return

        timestamp_a_sec: Optional[float] = self._motor_voltage_a_timestamp_sec
        timestamp_b_sec: Optional[float] = self._motor_voltage_b_timestamp_sec
        if timestamp_a_sec is None or timestamp_b_sec is None:
            return

        sample_skew_sec: float = abs(timestamp_a_sec - timestamp_b_sec)
        if sample_skew_sec > MAX_MOTOR_VOLTAGE_SKEW_SECS:
            return

        # The H-bridge drives the motor from the difference between its two
        # output wires, so the signed motor voltage is wire A minus wire B
        motor_voltage: float = self._motor_voltage_a - self._motor_voltage_b

        # The old supply-times-duty estimate was removed because these fields
        # now report the measured H-bridge output from the ADC stream
        self._motor_voltage = motor_voltage

        # Update the base variance from the measured differential voltage
        # stream before inflating it for asynchronous sampling uncertainty
        if not self._motor_voltage_initialized:
            self._motor_voltage_mu = motor_voltage
            self._motor_voltage_base_var = 0.0
            self._motor_voltage_initialized = True
        else:
            err: float = motor_voltage - self._motor_voltage_mu
            self._motor_voltage_mu += STDDEV_BETA * err
            self._motor_voltage_base_var = (1 - STDDEV_BETA) * (
                self._motor_voltage_base_var + STDDEV_BETA * err * err
            )

        self._motor_voltage_base_var = max(self._motor_voltage_base_var, 0.0)

        # Asynchronous A/B ADC sampling adds uncertainty because the
        # differential voltage combines two measurements captured at different
        # times while the H-bridge output may be changing
        slope_a: Optional[float] = self._estimate_slope(
            self._motor_voltage_a,
            self._motor_voltage_a_timestamp_sec,
            self._prev_motor_voltage_a,
            self._prev_motor_voltage_a_timestamp_sec,
        )
        slope_b: Optional[float] = self._estimate_slope(
            self._motor_voltage_b,
            self._motor_voltage_b_timestamp_sec,
            self._prev_motor_voltage_b,
            self._prev_motor_voltage_b_timestamp_sec,
        )

        motor_voltage_var: float = self._motor_voltage_base_var

        # Inflate the uncertainty using the local slope mismatch so fast
        # changes and larger A/B skew produce a larger measurement sigma
        if slope_a is not None and slope_b is not None:
            skew_sigma: float = 0.5 * abs(slope_a - slope_b) * sample_skew_sec
            motor_voltage_var += skew_sigma * skew_sigma

        self._motor_voltage_stddev = math.sqrt(max(motor_voltage_var, 0.0))

    def _update_motor_voltage_channel_a(
        self,
        voltage: float,
        timestamp_sec: float,
    ) -> None:
        self._prev_motor_voltage_a = self._motor_voltage_a
        self._prev_motor_voltage_a_timestamp_sec = self._motor_voltage_a_timestamp_sec
        self._motor_voltage_a = voltage
        self._motor_voltage_a_timestamp_sec = timestamp_sec
        self._motor_voltage_a_initialized = True

        self._update_motor_voltage()

    def _update_motor_voltage_channel_b(
        self,
        voltage: float,
        timestamp_sec: float,
    ) -> None:
        self._prev_motor_voltage_b = self._motor_voltage_b
        self._prev_motor_voltage_b_timestamp_sec = self._motor_voltage_b_timestamp_sec
        self._motor_voltage_b = voltage
        self._motor_voltage_b_timestamp_sec = timestamp_sec
        self._motor_voltage_b_initialized = True

        self._update_motor_voltage()

    def _set_analog_mode(self, analog_pin: int, analog_mode: AnalogMode) -> bool:
        # Create message
        vss_analog_svc = SetAnalogModeSvc.Request()
        vss_analog_svc.analog_pin = analog_pin
        vss_analog_svc.analog_mode = RosTranslator.analog_mode_to_ros(analog_mode)

        # Call service
        future: rclpy.task.Future = self._set_analog_mode_client.call_async(
            vss_analog_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _set_digital_mode(self, digital_pin: int, digital_mode: DigitalMode) -> bool:
        # Create message
        motor_pwm_svc = SetDigitalModeSvc.Request()
        motor_pwm_svc.digital_pin = digital_pin
        motor_pwm_svc.digital_mode = RosTranslator.digital_mode_to_ros(digital_mode)

        # Call service
        future: rclpy.task.Future = self._set_digital_mode_client.call_async(
            motor_pwm_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _on_analog_reading(self, analog_reading_msg: AnalogReadingMsg) -> None:
        analog_pin: int = analog_reading_msg.analog_pin
        analog_value: float = analog_reading_msg.analog_value
        timestamp_sec: float = self._time_msg_to_sec(analog_reading_msg.header.stamp)

        # Translate analog value
        analog_voltage: float = analog_value * AREF_VOLTAGE

        if analog_pin == self._vss_pin:
            # Apply voltage divider formula
            supply_voltage: float = analog_voltage * (VSS_R1 + VSS_R2) / VSS_R2

            # Adjust for empirical gain
            supply_voltage *= VIN_GAIN

            # Record state
            self._supply_voltage = supply_voltage

            # Update EWMA statistics
            if not self._supply_voltage_initialized:
                self._supply_voltage_mu = supply_voltage
                self._supply_voltage_var = 0.0
                self._supply_voltage_initialized = True
            else:
                err: float = supply_voltage - self._supply_voltage_mu
                self._supply_voltage_mu += STDDEV_BETA * err
                self._supply_voltage_var = (1 - STDDEV_BETA) * (
                    self._supply_voltage_var + STDDEV_BETA * err * err
                )

            self._supply_voltage_var = max(self._supply_voltage_var, 0.0)
            self._supply_voltage_stddev = math.sqrt(self._supply_voltage_var)

        elif analog_pin == self._motor_current_pin:
            # TODO: Apply Vref and Gain to get current
            motor_current: float = analog_voltage

            # Record state
            self._motor_current = motor_current
        elif analog_pin == self._motor_voltage_a_pin:
            motor_voltage_a: float = self._decode_divider_voltage(
                analog_voltage,
                MOTOR_VOLTAGE_A_R1,
                MOTOR_VOLTAGE_A_R2,
            )

            self._update_motor_voltage_channel_a(motor_voltage_a, timestamp_sec)
        elif analog_pin == self._motor_voltage_b_pin:
            motor_voltage_b: float = self._decode_divider_voltage(
                analog_voltage,
                MOTOR_VOLTAGE_B_R1,
                MOTOR_VOLTAGE_B_R2,
            )

            self._update_motor_voltage_channel_b(motor_voltage_b, timestamp_sec)

    def _on_digital_reading(self, digital_reading_msg: DigitalReadingMsg) -> None:
        # Translate parameters
        digital_pin: int = digital_reading_msg.digital_pin
        digital_value: bool = digital_reading_msg.digital_value == 1

        if digital_pin == self._motor_ff1_pin:
            if digital_value != self._motor_ff1_state:
                # Increment count on high edge
                if digital_value:
                    self._motor_ff1_count += 1

                # Record state
                self._motor_ff1_state = digital_value
        elif digital_pin == self._motor_ff2_pin:
            if digital_value != self._motor_ff2_state:
                # Increment count on high edge
                if digital_value:
                    self._motor_ff2_count += 1

                # Record state
                self._motor_ff2_state = digital_value
