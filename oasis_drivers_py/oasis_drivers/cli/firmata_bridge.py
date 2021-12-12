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
# Bridge from ROS 2 to Firmata running on an AVR processor.
#
# Dependencies:
#
#   * pymata-express
#

import rclpy

from oasis_drivers.nodes.firmata_bridge_node import FirmataBridgeNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FirmataBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

    rclpy.shutdown()
