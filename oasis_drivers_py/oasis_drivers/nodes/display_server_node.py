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
from oasis_msgs.srv import SetDisplay as SetDisplaySvc


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "display_server"

SET_DISPLAY_SERVICE = "set_display"


################################################################################
# ROS node
################################################################################


class DisplayServerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Node state
        self._has_dpms: bool = False
        self._has_brightness: bool = False

        # Services
        self._set_display_service: rclpy.service.Service = self.create_service(
            srv_type=SetDisplaySvc,
            srv_name=SET_DISPLAY_SERVICE,
            callback=self._handle_set_display,
        )

        self.get_logger().info("Display server initialized")

        # Do initial detection
        try:
            DisplayServer.ensure_dpms()
            self._has_dpms = True
        except Exception as err:
            self.get_logger().error(f"DPMS check failed: {err}")

        try:
            self.get_logger().info(DisplayServer.detect_displays())
            self._has_brightness = True
        except Exception as err:
            self.get_logger().error(f"DDC/CI check failed: {err}")

    def stop(self) -> None:
        self.get_logger().info("Display server deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()

    def _handle_set_display(
        self, request: SetDisplaySvc.Request, response: SetDisplaySvc.Response
    ) -> SetDisplaySvc.Response:
        """
        Handle the SetDisplay service request.
        """
        # Translate parameters
        power_mode: bool = request.dpms_mode == PowerModeMsg.ON

        try:
            if self._has_dpms:
                self.get_logger().debug(f"Setting DPMS to {request.dpms_mode}")
                DisplayServer.set_dpms(power_mode)
            if power_mode and self._has_brightness:
                self.get_logger().debug(f"Setting brightness to {request.brightness}")
                DisplayServer.set_brightness(request.brightness)
        except Exception as err:
            self.get_logger().error(f"Display server error: {err}")

        return response
