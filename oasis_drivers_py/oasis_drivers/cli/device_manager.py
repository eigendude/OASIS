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
# Manage devices via udev
#
# Dependencies:
#
#  * pyudev
#

import rclpy

from oasis_drivers.nodes.device_manager_node import DeviceManagerNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    manager = DeviceManagerNode()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()

    rclpy.shutdown()
