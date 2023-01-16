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

from oasis_msgs.msg import PowerMode as PowerModeMsg
from oasis_msgs.srv import PowerControl as PowerControlSvc


################################################################################
# ROS parameters
################################################################################


# Service clients
CLIENT_POWER_CONTROL = "power_control"


################################################################################
# Manager
################################################################################


class DisplayManager:
    """
    A manager for sensing and reporting MCU memory
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        """
        Initialize resources.
        """
        # Initialize manager state
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Service clients
        self._power_control_client: rclpy.client.Client = self._node.create_client(
            srv_type=PowerControlSvc, srv_name=CLIENT_POWER_CONTROL
        )

    def initialize(self) -> bool:
        self._logger.debug("Waiting for power control service...")
        self._set_sampling_interval_client.wait_for_service()

        self._logger.debug("Starting display manager configuration")

        # Turn display off
        self._logger.debug(f"Setting display power to {PowerModeMsg.OFF}")
        if not self._control_power(PowerModeMsg.OFF):
            return False

        self._logger.info("Sampling manager initialized successfully")

        return True

    def deinitialize(self) -> None:
        self._logger.debug("Deinitializing display manager")

        # Turn display back on
        self._logger.debug(f"Setting display power to {PowerModeMsg.ON}")
        if self._control_power(PowerModeMsg.ON):
            self._logger.info("Display manager deinitialized")
        else:
            self._logger.error("Error deinitializing display manager")

    def _control_power(self, power_mode: str) -> bool:
        # Create message
        power_control_svc = PowerControlSvc.Request()
        power_control_svc.power_mode = power_mode

        # Call service
        future: asyncio.Future = self._power_control_client.call_async(
            power_control_svc
        )

        # Wait for result
        rclpy.spin_until_future_complete(self._node, future)
        if future.result() is None:
            self._logger.error(f"Exception while calling service: {future.exception()}")
            return False

        return True
