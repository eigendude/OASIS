################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Control power to displays. Currently only works for laptops.
#
# Dependencies:
#
#  * vbetool - Needs setuid, run "sudo chmod u+s /usr/sbin/vbetool"
#

import rclpy

from oasis_drivers.nodes.display_server_node import DisplayServerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DisplayServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

    rclpy.shutdown()
