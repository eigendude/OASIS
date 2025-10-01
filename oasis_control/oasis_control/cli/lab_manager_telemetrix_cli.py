################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

import rclpy

from oasis_control.nodes.lab_manager_telemetrix_node import LabManagerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = LabManagerNode()
    if node.initialize():
        rclpy.spin(node)

        # Destroy the node explicitly. Problems can occur when the garbage
        # collector automatically destroys the node object after ROS has
        # shut down.
        node.destroy_node()
