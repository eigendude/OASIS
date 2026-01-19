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
ROS clock implementation for AHRS localization
"""

from __future__ import annotations

import rclpy.node

from oasis_control.localization.ahrs.ahrs_clock import AhrsClock
from oasis_control.localization.ahrs.ahrs_ros_conversions import ahrs_time_from_ros
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


class RosAhrsClock(AhrsClock):
    """
    ROS clock implementation backed by a node's time source
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        self._node: rclpy.node.Node = node

    def now(self) -> AhrsTime:
        return ahrs_time_from_ros(self._node.get_clock().now().to_msg())
