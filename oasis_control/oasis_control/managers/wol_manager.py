################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a computer that supports Wake-On-LAN (WOL)
#

from typing import Optional

import rclpy.client
import rclpy.node
import rclpy.task

from oasis_msgs.srv import GetMACAddress as GetMACAddressSvc
from oasis_msgs.srv import WoLCommand as WoLCommandSvc


################################################################################
# ROS parameters
################################################################################


# Service clients
CLIENT_GET_MAC_ADDRESS = "get_mac_address"
CLIENT_WOL = "wol"

# Timeout for ROS service calls
TIMEOUT_SECS: float = 5.0

################################################################################
# ROS node
################################################################################


class WolManager:
    """
    A manager for Wake-On-LAN (WOL) functionality.
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        hostname: str,
    ) -> None:
        """
        Initialize resources.

        :param node: The ROS node to use for communication
        :param hostname: The hostname or IP of the computer to manage, e.g.
                         "homeassistant.local" or "192.168.1.42"
        """
        # Initialize ROS parameters
        self._node: rclpy.node.Node = node
        self._logger = node.get_logger()

        # Initialize computer config
        self._hostname: str = hostname
        self._mac_address: Optional[str] = None

        # Futures for async service calls
        self._mac_address_future: Optional[rclpy.task.Future] = None
        self._wol_future: Optional[rclpy.task.Future] = None

        # Service clients
        self._get_mac_address_client: rclpy.client.Client = self._node.create_client(
            srv_type=GetMACAddressSvc,
            srv_name=CLIENT_GET_MAC_ADDRESS,
        )
        self._wol_client: rclpy.client.Client = self._node.create_client(
            srv_type=WoLCommandSvc,
            srv_name=CLIENT_WOL,
        )

    def initialize(self, service_timeout_secs: Optional[float] = None) -> bool:
        self._logger.debug("Waiting for WoL services")

        self._logger.debug("  - Waiting for get_mac_address...")
        if not self._get_mac_address_client.wait_for_service(service_timeout_secs):
            self._logger.error(
                f"Failed to connect to get_mac_address service after {service_timeout_secs}s"
            )
            return False

        self._logger.debug("  - Waiting for wol...")
        if not self._wol_client.wait_for_service(service_timeout_secs):
            self._logger.error(
                f"Failed to connect to wol service after {service_timeout_secs}s"
            )
            return False

        # Get the MAC address
        self._logger.debug(f"Getting MAC address for {self._hostname}")

        get_mac_svc = GetMACAddressSvc.Request()
        get_mac_svc.hostname = self._hostname

        # Call service asynchronously
        self._mac_address_future = self._get_mac_address_client.call_async(get_mac_svc)

        return True

    def send_wol(self) -> bool:
        if self._mac_address is None:
            # Wait for result
            mac_address_future = self._mac_address_future
            if mac_address_future is None:
                self._logger.error("MAC address future was not initialized")
                return False

            rclpy.spin_until_future_complete(
                self._node, mac_address_future, None, TIMEOUT_SECS
            )

            mac_address_response = mac_address_future.result()
            if mac_address_response is None:
                self._logger.error(
                    f"Exception while calling service: {mac_address_future.exception()}"
                )
                return False

            # Record MAC address
            self._mac_address = mac_address_response.mac_address

            # Log MAC address
            if self._mac_address == "":
                self._logger.error(f"Failed to get MAC address for {self._hostname}")
                return False
            else:
                self._logger.info(
                    f"MAC address for {self._hostname} is {self._mac_address}"
                )

        self._logger.info(
            f"Sending Wake-On-LAN to {self._hostname} ({self._mac_address})"
        )

        wol_svc = WoLCommandSvc.Request()
        wol_svc.mac_address = self._mac_address

        # Call service asynchronously
        self._wol_future = self._wol_client.call_async(wol_svc)

        # Wait for result
        wol_future = self._wol_future
        if wol_future is None:
            self._logger.error("WoL future was not initialized")
            return False
        rclpy.spin_until_future_complete(self._node, wol_future, None, TIMEOUT_SECS)

        wol_response = wol_future.result()
        if wol_response is None:
            self._logger.error(
                f"Exception while calling service: {wol_future.exception()}"
            )
            return False

        return True
