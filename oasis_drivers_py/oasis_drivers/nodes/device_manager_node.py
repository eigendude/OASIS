################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy.node
import rclpy.service

from oasis_drivers.system.device_manager import DeviceManager


################################################################################
# ROS parameters
################################################################################


NODE_NAME = "device_manager"


################################################################################
# ROS node
################################################################################


class DeviceManagerNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Initialize resources.
        """
        super().__init__(NODE_NAME)

        # Initialize device manager
        self._device_manager: DeviceManager = DeviceManager(self.get_logger())

        # Start device manager
        self._device_manager.start()

        self.get_logger().info("Device manager initialized")

    def stop(self) -> None:
        # Stop device manager
        self._device_manager.stop()

        # Join device manager
        self._device_manager.join()

        self.get_logger().info("Device manager deinitialized")

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        self.destroy_node()
