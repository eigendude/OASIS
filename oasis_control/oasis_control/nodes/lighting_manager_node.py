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


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "lighting_manager"


################################################################################
# ROS node
################################################################################


class LightingManagerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Subsystems
        self._display_manager: DisplayManager = DisplayManager(self)
        self._lighting_manager: LightingManager = LightingManager(self)

        self.get_logger().info("Lighting manager initialized")
