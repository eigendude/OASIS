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

import argparse

import rclpy

from oasis_drivers.nodes.tilt_sensor_node import TiltSensorNode


################################################################################
# ROS entry point
################################################################################


def _parse_args(args=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start the tilt sensor node")
    parser.add_argument(
        "--robot",
        default="falcon",
        help="Robot name used for default topic names",
    )
    parser.add_argument(
        "--namespace",
        default="oasis",
        help="Namespace used for default topic names",
    )
    options, _ = parser.parse_known_args(args=args)
    return options


def main(args=None) -> None:
    rclpy.init(args=args)

    options = _parse_args(args=args)

    node = TiltSensorNode(robot_name=options.robot, namespace=options.namespace)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
