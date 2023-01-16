#!/usr/bin/env python3
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Display manager for laptops with an external display
#

import rclpy.node
from rclpy.logging import LoggingSeverity

from oasis_control.managers.display_manager import DisplayManager


################################################################################
# ROS parameters
################################################################################


ROS_NAMESPACE = "oasis"

NODE_NAME = "display_manager"

POWER_CONTROL_SERVICE = "power_control"


################################################################################
# ROS node
################################################################################


class DisplayManagerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Enable debug logging
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Subsystems
        self._display_manager: DisplayManager = DisplayManager(self)

    def initialize(self):
        # Initialize subsystems
        self._display_manager.initialize()

    def deinitialize(self):
        # Deinitialize subsystems
        self._display_manager.deinitialize()
