################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Control reading and writing to UPS (uninterruptible power supply) devices.
#
# Dependencies:
#
#  * nut (sudo apt install nut)
#

import rclpy

from oasis_drivers.nodes.ups_server_node import UpsServerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = UpsServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

    rclpy.shutdown()
