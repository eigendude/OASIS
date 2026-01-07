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
ROS conversions for EKF time types
"""

from builtin_interfaces.msg import Time as RosTime

from oasis_control.localization.ekf.ekf_types import EkfTime


def ros_time_to_ekf_time(msg: RosTime) -> EkfTime:
    return EkfTime(sec=msg.sec, nanosec=msg.nanosec)


def ekf_time_to_ros_time(t: EkfTime) -> RosTime:
    stamp: RosTime = RosTime()
    stamp.sec = t.sec
    stamp.nanosec = t.nanosec
    return stamp
