################################################################################
#
#  Copyright (C) 2016-2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Manage WiFi devices
#
# Dependencies:
#
#  * iw
#

import rclpy

from oasis_drivers.nodes.wifi_manager_node import WiFiManagerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    manager = WiFiManagerNode()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly. Problems can occur when the garbage
    # collector automatically destroys the node object after ROS has
    # shut down.
    manager.destroy_node()

    rclpy.shutdown()
