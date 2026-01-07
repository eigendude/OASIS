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
ROS entry point for EKF localization
"""

from typing import Optional

import rclpy

from oasis_control.nodes.ekf_localizer_node import EkfLocalizerNode


################################################################################
# ROS entry point
################################################################################


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)

    node: EkfLocalizerNode = EkfLocalizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()
