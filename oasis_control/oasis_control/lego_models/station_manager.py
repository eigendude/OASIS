################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a LEGO train station's microcontroller
#

import rclpy.client
import rclpy.node
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.task

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


VSS_PIN: int = 0  # A0
VSS_R1: float = 26.62  # KΩ
VSS_R2: float = 9.83  # KΩ
# External AREF is tied to a 5.03 V regulator for stable ADC scaling
AREF_VOLTAGE: float = 5.03  # V
MOTOR_PWM_PIN: int = 5  # D5
MOTOR_DIR_PIN: int = 4  # D4
MOTOR_FF1_PIN: int = 8  # D8
MOTOR_FF2_PIN: int = 7  # D7
MOTOR_CURRENT_PIN: int = 1  # A1


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
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Construction parameters
        self._node = node

        # Initialize hardware state
        self._supply_voltage: float = 0.0
        self._motor_voltage: float = 0.0
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
            depth=1,
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
    def motor_voltage(self) -> float:
        return self._motor_voltage

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
        self._node.get_logger().debug(f"Enabling VSS on A{VSS_PIN}")
        if not self._set_analog_mode(VSS_PIN, AnalogMode.INPUT):
            return False

        #
        # Pololu md07a motor driver 18v15
        #

        # Motor PWM
        self._node.get_logger().debug(f"Enabling motor PWM on D{MOTOR_PWM_PIN}")
        if not self._set_digital_mode(MOTOR_PWM_PIN, DigitalMode.PWM):
            return False

        # Motor DIR
        self._node.get_logger().debug(f"Enabling motor DIR on D{MOTOR_DIR_PIN}")
        if not self._set_digital_mode(MOTOR_DIR_PIN, DigitalMode.OUTPUT):
            return False

        # Motor FF1
        self._node.get_logger().debug(f"Enabling motor FF1 on D{MOTOR_FF1_PIN}")
        if not self._set_digital_mode(MOTOR_FF1_PIN, DigitalMode.INPUT):
            return False

        # Motor FF2
        self._node.get_logger().debug(f"Enabling motor FF2 on D{MOTOR_FF2_PIN}")
        if not self._set_digital_mode(MOTOR_FF2_PIN, DigitalMode.INPUT):
            return False

        #
        # Sparkfun ACS712 current sensor
        #

        # Output voltage (VO)
        self._node.get_logger().debug(
            f"Enabling current sensor VO on A{MOTOR_CURRENT_PIN}"
        )
        if not self._set_analog_mode(MOTOR_CURRENT_PIN, AnalogMode.INPUT):
            return False

        self._node.get_logger().info("Station manager initialized successfully")

        return True

    def set_motor_direction(self, reverse: bool) -> None:
        """Publish command for motor direction"""
        dir_cmd = DigitalWriteCommandMsg()
        dir_cmd.digital_pin = MOTOR_DIR_PIN
        dir_cmd.digital_value = reverse

        self._motor_dir_cmd_pub.publish(dir_cmd)

    def set_motor_pwm(self, target_magnitude: float, reverse: bool) -> None:
        """Publish command for motor PWM"""
        pwm_cmd = PWMWriteCommandMsg()
        pwm_cmd.digital_pin = MOTOR_PWM_PIN
        pwm_cmd.duty_cycle = target_magnitude

        self._motor_pwm_cmd_pub.publish(pwm_cmd)

        # Update state
        self._motor_voltage = (
            self._supply_voltage * target_magnitude * (-1 if reverse else 1)
        )

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

        # Translate analog value
        analog_voltage: float = analog_value * AREF_VOLTAGE

        if analog_pin == VSS_PIN:
            # Apply voltage divider formula
            supply_voltage: float = analog_voltage * (VSS_R1 + VSS_R2) / VSS_R2

            # Record state
            self._supply_voltage = supply_voltage

        elif analog_pin == MOTOR_CURRENT_PIN:
            # TODO: Apply Vref and Gain to get current
            motor_current: float = analog_voltage

            # Record state
            self._motor_current = motor_current

    def _on_digital_reading(self, digital_reading_msg: DigitalReadingMsg) -> None:
        # Translate parameters
        digital_pin: int = digital_reading_msg.digital_pin
        digital_value: bool = digital_reading_msg.digital_value == 1

        if digital_pin == MOTOR_FF1_PIN:
            if digital_value != self._motor_ff1_state:
                # Increment count on high edge
                if digital_value:
                    self._motor_ff1_count += 1

                # Record state
                self._motor_ff1_state = digital_value
        elif digital_pin == MOTOR_FF2_PIN:
            if digital_value != self._motor_ff2_state:
                # Increment count on high edge
                if digital_value:
                    self._motor_ff2_count += 1

                # Record state
                self._motor_ff2_state = digital_value
