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
# ROS node for interacting with psutil, the cross-platform library for
# retrieving process and system information.
#
# Reports the following data:
#
#   - CPU percent
#   - CPU frequency
#   - CPU temperature
#   - Memory utilization
#   - Disk utilization
#   - Network I/O
#
# Reports the following state:
#
#   - CPU counts
#   - CPU frequencies
#   - Memory configuration
#   - Disk configuration
#   - Network addresses
#

import rclpy

from oasis_drivers.nodes.system_monitor_node import SystemMonitorNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    monitor = SystemMonitorNode()
    rclpy.spin(monitor)

    # Destroy the node explicitly. Problems can occur when the garbage
    # collector automatically destroys the node object after ROS has
    # shut down.
    monitor.destroy_node()

    rclpy.shutdown()
