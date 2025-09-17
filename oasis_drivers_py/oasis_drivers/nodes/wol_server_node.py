################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy
import rclpy.node
import rclpy.service

from oasis_drivers.network.wol_server import WolServer
from oasis_msgs.srv import GetMACAddress as GetMACAddressSvc
from oasis_msgs.srv import WoLCommand as WoLCommandSvc


################################################################################
# ROS parameters
################################################################################


# The default node name
NODE_NAME = "wol_server"

# ROS services
GET_MAC_ADDRESS_SERVICE = "get_mac_address"
WOL_SERVICE = "wol"


################################################################################
# ROS node
################################################################################


class WolServerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # ROS services
        self._get_mac_service: rclpy.service.Service = self.create_service(
            srv_type=GetMACAddressSvc,
            srv_name=GET_MAC_ADDRESS_SERVICE,
            callback=self._handle_get_mac_address,
        )
        self._wol_command_service: rclpy.service.Service = self.create_service(
            srv_type=WoLCommandSvc,
            srv_name=WOL_SERVICE,
            callback=self._handle_wol_command,
        )

        self.get_logger().info("WoL server initialized")

    def stop(self) -> None:
        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

        self.get_logger().info("WoL server deinitialized")

    def _handle_get_mac_address(
        self, request: GetMACAddressSvc.Request, response: GetMACAddressSvc.Response
    ) -> GetMACAddressSvc.Response:
        """
        Look up request.hostname → IP, ping to populate ARP cache, then read
        `ip neigh show` to extract the MAC.
        """
        hostname: str = request.hostname

        self.get_logger().info(f"Getting MAC address for '{hostname}'")

        response.mac_address = WolServer.get_mac_address(hostname)

        return response

    def _handle_wol_command(
        self, request: WoLCommandSvc.Request, response: WoLCommandSvc.Response
    ) -> WoLCommandSvc.Response:
        mac_address: str = request.mac_address

        self.get_logger().info(f"Sending WoL command to '{mac_address}'")

        try:
            WolServer.send_wol(mac_address)

            self.get_logger().info(
                f"WoL command to '{mac_address}' executed successfully"
            )
        except Exception as err:
            self.get_logger().error(
                f"Failed to execute WoL command to '{mac_address}': {err}"
            )

        return response
