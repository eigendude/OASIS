################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manager for a computer that supports Wake-On-LAN (WOL)
#

import re
from typing import Optional
from typing import cast

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

# Colon-separated IEEE 802 MAC address format accepted by the WoL service
MAC_ADDRESS_PATTERN: re.Pattern[str] = re.compile(
    r"^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$"
)

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
        self._get_mac_address_client: rclpy.client.Client[
            GetMACAddressSvc.Request, GetMACAddressSvc.Response
        ] = self._node.create_client(
            srv_type=GetMACAddressSvc,
            srv_name=CLIENT_GET_MAC_ADDRESS,
        )
        self._wol_client: rclpy.client.Client[
            WoLCommandSvc.Request, WoLCommandSvc.Response
        ] = self._node.create_client(
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

            mac_address_response_object = self._wait_for_service_response(
                future=mac_address_future,
                service_name=CLIENT_GET_MAC_ADDRESS,
                timeout_context=f"getting MAC address for {self._hostname}",
                service_context=f"for {self._hostname}",
            )
            if mac_address_response_object is None:
                return False

            mac_address_response: GetMACAddressSvc.Response = cast(
                GetMACAddressSvc.Response,
                mac_address_response_object,
            )

            # Record MAC address
            self._mac_address = mac_address_response.mac_address

            # Log MAC address
            if self._mac_address.strip() == "":
                self._logger.error(
                    "get_mac_address service returned an empty MAC address "
                    f"for {self._hostname}"
                )
                return False
            elif not self._is_valid_mac_address(self._mac_address):
                self._logger.error(
                    "get_mac_address service returned an invalid MAC address "
                    f"for {self._hostname}: {self._mac_address}"
                )
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
        wol_response = self._wait_for_service_response(
            future=wol_future,
            service_name=CLIENT_WOL,
            timeout_context=(
                f"sending Wake-on-LAN to {self._hostname} ({self._mac_address})"
            ),
            service_context=f"for {self._hostname} ({self._mac_address})",
        )
        if wol_response is None:
            return False

        return True

    def _wait_for_service_response(
        self,
        future: rclpy.task.Future,
        service_name: str,
        timeout_context: str,
        service_context: str,
    ) -> Optional[object]:
        # A timeout can return before the future is done. In that case
        # future.exception() can still be None, so check done() first.
        rclpy.spin_until_future_complete(
            node=self._node,
            future=future,
            timeout_sec=TIMEOUT_SECS,
        )

        if not future.done():
            self._logger.error(f"Timed out after {TIMEOUT_SECS}s {timeout_context}")
            return None

        try:
            response: object = future.result()
        except Exception as ex:
            self._logger.error(f"{service_name} service failed {service_context}: {ex}")
            return None

        if response is None:
            self._logger.error(
                f"{service_name} service completed with no response {service_context}"
            )
            return None

        return response

    @staticmethod
    def _is_valid_mac_address(mac_address: str) -> bool:
        return MAC_ADDRESS_PATTERN.fullmatch(mac_address) is not None
