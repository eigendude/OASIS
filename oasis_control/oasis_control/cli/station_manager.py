################################################################################
#
#  Copyright (C) 2022 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy

from oasis_control.nodes.station_manager_node import StationManagerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    manager = StationManagerNode()
    if manager.initialize():
        rclpy.spin(manager)

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        manager.destroy_node()

    rclpy.shutdown()
