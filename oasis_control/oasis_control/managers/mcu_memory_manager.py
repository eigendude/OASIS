################################################################################
#
#  Copyright (C) 2022 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a mcu that reports memory statistics
#

import asyncio

import rclpy.client
import rclpy.node
import rclpy.qos
import rclpy.subscription

from oasis_msgs.msg import MCUMemory as MCUMemoryMsg
from oasis_msgs.srv import ReportMCUMemory as ReportMCUMemorySvc


# from std_msgs.msg import Header as HeaderMsg


################################################################################
# Hardware configuration
################################################################################


# Timing parameters
REPORT_MCU_MEMORY_PERIOD_SECS: float = 10.0  # RAM utilization doesn't change


################################################################################
# ROS parameters
################################################################################


# Subscribers
SUBSCRIBE_MCU_MEMORY = "mcu_memory"

# Service clients
CLIENT_REPORT_MCU_MEMORY = "report_mcu_memory"


################################################################################
# Manager
################################################################################


class McuMemoryManager:
    """
    A manager for sensing and reporting MCU memory
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Initialize manager state
        self._node: rclpy.node.Node = node

        # Initialize hardware state
        self._total_ram: int = 0
        self._ram_utilization: float = 0.0

        # Reliable listener QOS profile for subscribers
        qos_profile: rclpy.qos.QoSPresetProfile = (
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        # Subscribers
        self._mcu_memory_sub: rclpy.subscription.Subscription = (
            self._node.create_subscription(
                msg_type=MCUMemoryMsg,
                topic=SUBSCRIBE_MCU_MEMORY,
                callback=self._on_mcu_memory,
                qos_profile=qos_profile,
            )
        )

        # Service clients
        self._report_mcu_memory_client: rclpy.client.Client = self._node.create_client(
            srv_type=ReportMCUMemorySvc, srv_name=CLIENT_REPORT_MCU_MEMORY
        )

    @property
    def total_ram(self) -> int:
        return self._total_ram

    @property
    def ram_utilization(self) -> float:
        return self._ram_utilization

    def initialize(self) -> bool:
        self._node.get_logger().debug("Waiting for MCU memory service...")
        self._report_mcu_memory_client.wait_for_service()

        self._node.get_logger().debug("Starting MCU memory configuration")

        # Memory reporting
        self._node.get_logger().debug("Enabling MCU memory reporting")
        if not self._report_mcu_memory(REPORT_MCU_MEMORY_PERIOD_SECS):
            return False

        self._node.get_logger().info("MCU memory manager initialized successfully")

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
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._node.get_logger().error(
                f"Exception while calling service: {future.exception()}"
            )
            return False

        return True

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
