################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Bridge from ROS 2 to Telemetrix server running on a microcontroller.
#
# Dependencies:
#
#   * telemetrix-aio
#

import rclpy

from oasis_drivers.nodes.telemetrix_bridge_node import TelemetrixBridgeNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TelemetrixBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
