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
# Lighting manager
#

import rclpy.node
from rclpy.logging import LoggingSeverity

from oasis_control.lighting.display_manager import DisplayManager
from oasis_control.lighting.lighting_manager import LightingManager


# from oasis_control.presence.presence_manager import PresenceManager


################################################################################
# ROS parameters
################################################################################


# ROS namespace
ROS_NAMESPACE: str = "oasis"

# Default node name
NODE_NAME: str = "home_manager"

# Parameters
SMART_DISPLAY_ZONES_PARAM: str = "smart_display_zones"
SMART_DISPLAY_PLUG_ID_PARAM: str = "smart_display_plug_id"


################################################################################
# ROS node
################################################################################


class HomeManagerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.get_logger().info("Home manager initializing...")

        # Declare parameters
        self.declare_parameter(
            SMART_DISPLAY_ZONES_PARAM,
            [""],  # Empty list is inferred as BYTE_ARRAY
        )
        self.declare_parameter(
            SMART_DISPLAY_PLUG_ID_PARAM,
            "",
        )

        # Read parameters
        smart_display_zones: list[str] = self.get_parameter(
            SMART_DISPLAY_ZONES_PARAM
        ).value
        smart_display_plug_id: str = self.get_parameter(
            SMART_DISPLAY_PLUG_ID_PARAM
        ).value

        # Subsystems
        self._display_manager: DisplayManager = DisplayManager(
            self, smart_display_zones, smart_display_plug_id
        )
        self._lighting_manager: LightingManager = LightingManager(self)
        # self._presence_manager: PresenceManager = PresenceManager(self)

        self.get_logger().info("Home manager initialized")
