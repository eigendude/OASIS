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
Clock abstraction for AHRS localization
"""

from __future__ import annotations

from builtin_interfaces.msg import Time as TimeMsg
import rclpy.node

from oasis_control.localization.ahrs.ahrs_conversions import ahrs_time_from_ros
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


class AhrsClock:
    """
    Clock abstraction for retrieving ROS time
    """

    def now_ros_time(self) -> TimeMsg:
        """
        Return the current ROS time
        """

        raise NotImplementedError

    def is_future_stamp(self, t_meas: AhrsTime, epsilon_sec: float) -> bool:
        """
        True when the measurement stamp is ahead of the wall clock
        """

        now: AhrsTime = ahrs_time_from_ros(self.now_ros_time())
        now_sec: float = float(now.sec) + float(now.nanosec) * 1e-9
        meas_sec: float = float(t_meas.sec) + float(t_meas.nanosec) * 1e-9

        return meas_sec > now_sec + epsilon_sec


class RosAhrsClock(AhrsClock):
    """
    ROS clock implementation backed by a node's time source
    """

    def __init__(self, node: rclpy.node.Node) -> None:
        self._node: rclpy.node.Node = node

    def now_ros_time(self) -> TimeMsg:
        return self._node.get_clock().now().to_msg()
