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
ROS entry point for estimating roll and pitch from IMU measurements.
"""

import rclpy

from oasis_drivers.nodes.tilt_sensor_node import TiltSensorNode


################################################################################
# ROS entry point
################################################################################


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TiltSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()
