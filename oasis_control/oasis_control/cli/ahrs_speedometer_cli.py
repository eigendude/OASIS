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
ROS entry point for the AHRS speedometer node
"""

from typing import Optional

import rclpy

from oasis_control.nodes.ahrs.ahrs_speedometer_node import AhrsSpeedometerNode


################################################################################
# ROS entry point
################################################################################


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)

    node: AhrsSpeedometerNode = AhrsSpeedometerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()
