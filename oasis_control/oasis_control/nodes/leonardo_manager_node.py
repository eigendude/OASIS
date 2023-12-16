################################################################################
#
#  Copyright (C) 2022-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# leonardo manager for NAS server
#

import asyncio
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
import rclpy.task
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_drivers.firmata.firmata_types import AnalogMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import AVRConstants as AVRConstantsMsg
from oasis_msgs.msg import LeonardoState as LeonardoStateMsg
from oasis_msgs.msg import MCUMemory as MCUMemoryMsg
from oasis_msgs.srv import ReportMCUMemory as ReportMCUMemorySvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc


################################################################################
# Hardware configuration
################################################################################


# Timing parameters
REPORT_MCU_MEMORY_PERIOD_SECS: float = 10.0  # RAM utilization doesn't change

# Reference voltage, in Volts
REFERENCE_VOLTAGE: float = 5.0

# Pins
POTENTIOMETER_PIN: int = 0  # A0


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "leonardo_manager"

PUBLISH_STATE_PERIOD_SECS = 0.5

# Publisher
PUBLISH_LEONARDO_STATE = "leonardo_state"

# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"
SUBSCRIBE_MCU_MEMORY = "mcu_memory"

# Service clients
CLIENT_REPORT_MCU_MEMORY = "report_mcu_memory"
CLIENT_SET_ANALOG_MODE = "set_analog_mode"


################################################################################
# ROS node
################################################################################


class LeonardoManagerNode(rclpy.node.Node):
    """
    A ROS node that manages the Arduino Leonardo attached to the NAS server.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Initialize hardware state
        self._potentiometer: float = 0.0
        self._total_ram: int = 0
        self._ram_utilization: float = 0.0

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._leonardo_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=LeonardoStateMsg,
            topic=PUBLISH_LEONARDO_STATE,
            qos_profile=qos_profile,
        )

        # Subscribers
        self._analog_reading_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=AnalogReadingMsg,
                topic=SUBSCRIBE_ANALOG_READING,
                callback=self._on_analog_reading,
                qos_profile=qos_profile,
            )
        )
        self._mcu_memory_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=MCUMemoryMsg,
                topic=SUBSCRIBE_MCU_MEMORY,
                callback=self._on_mcu_memory,
                qos_profile=qos_profile,
            )
        )

        # Service clients
        self._report_mcu_memory_client: rclpy.client.Client = self.create_client(
            srv_type=ReportMCUMemorySvc, srv_name=CLIENT_REPORT_MCU_MEMORY
        )
        self._set_analog_mode_client: rclpy.client.Client = self.create_client(
            srv_type=SetAnalogModeSvc, srv_name=CLIENT_SET_ANALOG_MODE
        )

        # Timer parameters
        self._publish_state_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        self.get_logger().debug("Waiting for Firmata services...")

        self._report_mcu_memory_client.wait_for_service()
        self._set_analog_mode_client.wait_for_service()

        self.get_logger().debug("Starting configuration")

        # Memory reporting
        self.get_logger().debug("Enabling MCU memory reporting")
        if not self._report_mcu_memory(REPORT_MCU_MEMORY_PERIOD_SECS):
            return False

        #
        # Potentiometer
        #

        # Vin
        self.get_logger().debug(f"Enabling potentiometer on A{POTENTIOMETER_PIN}")
        if not self._set_analog_mode(POTENTIOMETER_PIN, AnalogMode.INPUT):
            return False

        # Now that the manager is initialized, start the publishing timer
        self._publish_state_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        self.get_logger().info("Leonardo manager initialized successfully")

        return True

    def _report_mcu_memory(self, reporting_period_secs: float) -> bool:
        # Create message
        report_memory_svc = ReportMCUMemorySvc.Request()
        report_memory_svc.reporting_period_ms = int(reporting_period_secs * 1000)

        # Call service
        future: asyncio.Future = self._report_mcu_memory_client.call_async(
            report_memory_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _set_analog_mode(self, analog_pin: int, analog_mode: AnalogMode) -> bool:
        # Create message
        vss_analog_svc = SetAnalogModeSvc.Request()
        vss_analog_svc.analog_pin = analog_pin
        vss_analog_svc.analog_mode = self._translate_analog_mode(analog_mode)

        # Call service
        future: asyncio.Future = self._set_analog_mode_client.call_async(vss_analog_svc)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _on_analog_reading(self, analog_reading_msg: AnalogReadingMsg) -> None:
        analog_pin: int = analog_reading_msg.analog_pin
        reference_voltage: float = analog_reading_msg.reference_voltage
        analog_value: float = analog_reading_msg.analog_value

        # Translate analog value
        analog_voltage: float = analog_value * reference_voltage

        if analog_pin == POTENTIOMETER_PIN:
            # Record state
            self._potentiometer = analog_voltage

    def _on_mcu_memory(self, mcu_memory_msg: MCUMemoryMsg) -> None:
        # Translate parameters
        total_ram: int = mcu_memory_msg.total_ram
        free_ram: int = mcu_memory_msg.free_ram

        # Calculate utilization percent
        used_ram: int = total_ram - free_ram
        ram_utilization: float = 100.0 * used_ram / total_ram

        # Record state
        self._total_ram = total_ram
        self._ram_utilization = ram_utilization

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: LeonardoStateMsg = LeonardoStateMsg()
        msg.header = header
        msg.reference_voltage = REFERENCE_VOLTAGE
        msg.potentiometer = self._potentiometer
        msg.total_ram = self._total_ram
        msg.ram_utilization = self._ram_utilization

        self._leonardo_state_pub.publish(msg)

    @staticmethod
    def _translate_analog_mode(analog_mode: AnalogMode) -> int:
        return {
            AnalogMode.DISABLED: AVRConstantsMsg.ANALOG_DISABLED,
            AnalogMode.INPUT: AVRConstantsMsg.ANALOG_INPUT,
        }[analog_mode]
