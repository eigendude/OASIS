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

import rclpy

from oasis_automation.nodes.automation_manager_node import AutomationManagerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None):
    rclpy.init(args=args)

    manager = AutomationManagerNode()
    rclpy.spin(manager)

    # Destroy the node explicitly. Problems can occur when the garbage
    # collector automatically destroys the node object after ROS has
    # shut down.
    manager.destroy_node()

    rclpy.shutdown()
