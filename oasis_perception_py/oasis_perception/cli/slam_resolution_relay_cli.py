################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Launch the slam_resolution_relay node to mirror SLAM outputs between
resolution namespaces.
"""

import rclpy

from oasis_perception.nodes.slam_resolution_relay_node import SlamResolutionRelayNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SlamResolutionRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
