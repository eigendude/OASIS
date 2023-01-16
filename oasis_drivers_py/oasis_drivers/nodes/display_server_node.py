################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy.node
import rclpy.service

from oasis_drivers.display.display_server import DisplayServer
from oasis_msgs.msg import PowerMode as PowerModeMsg
from oasis_msgs.srv import PowerControl as PowerControlSvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "display_server"

POWER_CONTROL_SERVICE = "power_control"


################################################################################
# ROS node
################################################################################


class DisplayServerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Services
        self._power_control_service: rclpy.service.Service = self.create_service(
            srv_type=PowerControlSvc,
            srv_name=POWER_CONTROL_SERVICE,
            callback=self._handle_power_control,
        )

        self.get_logger().info("Display server initialized")

    def stop(self) -> None:
        self.get_logger().info("Display server deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

    def _handle_power_control(
        self, request: PowerControlSvc.Request, response: PowerControlSvc.Response
    ) -> PowerControlSvc.Response:
        power_mode: bool = request.power_mode == PowerModeMsg.ON

        DisplayServer.call_vbetool(power_mode, self.get_logger())

        return response
