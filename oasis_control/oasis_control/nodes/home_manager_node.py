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
from oasis_control.presence.presence_manager import PresenceManager


################################################################################
# ROS parameters
################################################################################


# ROS namespace
ROS_NAMESPACE = "oasis"

# Default node name
NODE_NAME = "home_manager"


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

        # Subsystems
        self._display_manager: DisplayManager = DisplayManager(self)
        self._lighting_manager: LightingManager = LightingManager(self)
        self._presence_manager: PresenceManager = PresenceManager(self)

        self.get_logger().info("Home manager initialized")
