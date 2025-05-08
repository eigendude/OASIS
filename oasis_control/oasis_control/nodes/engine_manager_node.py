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
# Manager for a LEGO train's engine
#

import asyncio
from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Header as HeaderMsg

from oasis_drivers.ros.ros_translator import RosTranslator
from oasis_drivers.telemetrix.telemetrix_types import AnalogMode
from oasis_msgs.msg import AnalogReading as AnalogReadingMsg
from oasis_msgs.msg import EngineState as EngineStateMsg
from oasis_msgs.msg import MCUMemory as MCUMemoryMsg
from oasis_msgs.msg import MCUString as MCUStringMsg
from oasis_msgs.srv import ReportMCUMemory as ReportMCUMemorySvc
from oasis_msgs.srv import SetAnalogMode as SetAnalogModeSvc
from oasis_msgs.srv import SetSamplingInterval as SetSamplingIntervalSvc


################################################################################
# Hardware configuration
################################################################################


# Timing parameters
REPORT_MCU_MEMORY_PERIOD_SECS: float = 10.0  # RAM utilization doesn't change

# Sampling interval, in ms
SAMPLING_INTERVAL_MS = 100

# Pins
VSS_PIN: int = 0  # A0


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "engine_manager"

PUBLISH_STATE_PERIOD_SECS = 0.1

# Publisher
PUBLISH_ENGINE_STATE = "engine_state"

# Subscribers
SUBSCRIBE_ANALOG_READING = "analog_reading"
SUBSCRIBE_MCU_MEMORY = "mcu_memory"
SUBSCRIBE_MCU_STRING = "mcu_string"

# Service clients
CLIENT_REPORT_MCU_MEMORY = "report_mcu_memory"
CLIENT_SET_ANALOG_MODE = "set_analog_mode"
CLIENT_SET_SAMPLING_INTERVAL = "set_sampling_interval"


################################################################################
# ROS node
################################################################################


class EngineManagerNode(rclpy.node.Node):
    """
    A ROS node that manages a LEGO train engine.
    """

    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Initialize hardware state
        self._total_ram: int = 0
        self._ram_utilization: float = 0.0
        self._supply_voltage: float = 0.0
        self._message: Optional[str] = None

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Publishers
        self._engine_state_pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=EngineStateMsg,
            topic=PUBLISH_ENGINE_STATE,
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
        self._mcu_string_sub: rclpy.subscription.Subscription = (
            self.create_subscription(
                msg_type=MCUStringMsg,
                topic=SUBSCRIBE_MCU_STRING,
                callback=self._on_mcu_string,
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
        self._set_sampling_interval_client: rclpy.client.Client = self.create_client(
            srv_type=SetSamplingIntervalSvc, srv_name=CLIENT_SET_SAMPLING_INTERVAL
        )

        # Timer parameters
        self._publish_state_timer: Optional[rclpy.node.Timer] = None

    def initialize(self) -> bool:
        self.get_logger().debug("Waiting for Telemetrix services")
        self.get_logger().debug("  - Waiting for report_mcu_memory...")
        self._report_mcu_memory_client.wait_for_service()
        self.get_logger().debug("  - Waiting for set_analog_mode...")
        self._set_analog_mode_client.wait_for_service()
        self.get_logger().debug("  - Waiting for set_sampling_interval...")
        self._set_sampling_interval_client.wait_for_service()

        self.get_logger().debug("Starting configuration")

        # Sampling interval
        self.get_logger().debug(
            f"Setting sampling interval to {SAMPLING_INTERVAL_MS} ms"
        )
        if not self._set_sampling_interval(SAMPLING_INTERVAL_MS):
            return False

        # Memory reporting
        self.get_logger().debug("Enabling MCU memory reporting")
        if not self._report_mcu_memory(REPORT_MCU_MEMORY_PERIOD_SECS):
            return False

        # Vss
        self.get_logger().debug(f"Enabling Vss on A{VSS_PIN}")
        if not self._set_analog_mode(VSS_PIN, AnalogMode.INPUT):
            return False

        # Now that the manager is initialized, start the publishing timer
        self._publish_state_timer = self.create_timer(
            timer_period_sec=PUBLISH_STATE_PERIOD_SECS, callback=self._publish_state
        )

        self.get_logger().info("Engine manager initialized successfully")

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
        vss_analog_svc.analog_mode = RosTranslator.analog_mode_to_ros(analog_mode)

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

    def _set_sampling_interval(self, sampling_interval_ms: int) -> bool:
        # Create message
        set_sampling_interval_svc = SetSamplingIntervalSvc.Request()
        set_sampling_interval_svc.sampling_interval_ms = sampling_interval_ms

        # Call service
        future: asyncio.Future = self._set_sampling_interval_client.call_async(
            set_sampling_interval_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

    def _on_analog_reading(self, analog_reading_msg: AnalogReadingMsg) -> None:
        # Translate parameters
        analog_pin: int = analog_reading_msg.analog_pin
        reference_voltage: float = analog_reading_msg.reference_voltage
        analog_value: float = analog_reading_msg.analog_value

        # Translate analog value
        analog_voltage: float = analog_value * reference_voltage

        if analog_pin == VSS_PIN:
            # Record state
            self._supply_voltage = analog_voltage

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

    def _on_mcu_string(self, mcu_string_msg: MCUStringMsg) -> None:
        # Translate parameters
        message: str = mcu_string_msg.message

        # Log message
        self.get_logger().debug(f"MCU: {message}")

        # Record state
        self._message = message

    def _publish_state(self) -> None:
        header = HeaderMsg()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = NODE_NAME  # TODO

        msg: EngineStateMsg = EngineStateMsg()
        msg.header = header
        msg.total_ram = self._total_ram
        msg.ram_utilization = self._ram_utilization
        msg.supply_voltage = self._supply_voltage
        msg.message = self._message if self._message else ""

        self._engine_state_pub.publish(msg)
