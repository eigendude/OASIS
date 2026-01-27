################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
ROS entry point for the AHRS node
"""

from typing import Optional

import rclpy

from oasis_control.nodes.ahrs_mounting_node import AhrsMountingNode


################################################################################
# ROS entry point
################################################################################


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)

    node: AhrsMountingNode = AhrsMountingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()
