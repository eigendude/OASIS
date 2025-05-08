################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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
        self._has_cec: bool = False

        # Services
        self._set_display_service: rclpy.service.Service = self.create_service(
            srv_type=SetDisplaySvc,
            srv_name=SET_DISPLAY_SERVICE,
            callback=self._handle_set_display,
        )

    def initialize(self) -> None:
        """
        Initialize the display server.
        """
        # Do initial detection
        try:
            DisplayServer.ensure_dpms()
            self._has_dpms = True
            self.get_logger().info("DisplayServer: Using DPMS via vbetool")
        except Exception as err:
            self.get_logger().warn(f"DisplayServer: DPMS check failed: {err}")

        try:
            self.get_logger().info(DisplayServer.detect_displays())
            self._has_brightness = True
            self.get_logger().info("DisplayServer: Using DDC/CI via ddcutil")
        except Exception as err:
            self.get_logger().warn(f"DisplayServer: DDC/CI check failed: {err}")

        try:
            DisplayServer.ensure_cec()
            self._has_cec = True
            self.get_logger().info("DisplayServer: Using CEC via cec-client")
        except Exception as err:
            self.get_logger().warn(f"DisplayServer: CEC check failed: {err}")

        self.get_logger().info("Display server initialized")

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

        self.get_logger().debug(
            f"Received set display request. mode: {request.dpms_mode}, brightness: {request.brightness}"
        )

        if self._has_dpms:
            self.get_logger().info(f"Setting DPMS to {request.dpms_mode}")
            try:
                DisplayServer.set_dpms(power_mode)
            except Exception as err:
                self.get_logger().error(f"Display server DPMS error: {err}")

        if self._has_brightness:
            self.get_logger().info(f"Setting brightness to {request.brightness}")
            try:
                DisplayServer.set_brightness(request.brightness)
            except Exception as err:
                self.get_logger().error(f"Display server brightness error: {err}")

        if self._has_cec:
            self.get_logger().info(f"Setting CEC to {request.dpms_mode}")
            try:
                DisplayServer.set_cec_power(power_mode)
            except Exception as err:
                self.get_logger().error(f"Display server CEC error: {err}")

        return response
