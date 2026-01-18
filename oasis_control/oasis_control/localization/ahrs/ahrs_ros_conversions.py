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
Conversion helpers between ROS messages and AHRS types
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from builtin_interfaces.msg import Time as TimeMsg

from oasis_control.localization.ahrs.ahrs_conversions import ahrs_time_from_pair
from oasis_control.localization.ahrs.ahrs_conversions import cov3x3_from_seq
from oasis_control.localization.ahrs.ahrs_conversions import normalize_ahrs_time
from oasis_control.localization.ahrs.ahrs_conversions import vector3_from_xyz
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


class Vector3Like(Protocol):
    """
    Protocol for geometry messages with x/y/z fields
    """

    x: float
    y: float
    z: float


def ahrs_time_from_ros(stamp: TimeMsg) -> AhrsTime:
    """
    Convert a ROS time stamp into an AHRS time
    """

    return ahrs_time_from_pair(sec=int(stamp.sec), nanosec=int(stamp.nanosec))


def ros_time_from_ahrs(t: AhrsTime) -> TimeMsg:
    """
    Convert an AHRS time into a ROS time stamp
    """

    normalized: AhrsTime = normalize_ahrs_time(t)

    stamp: TimeMsg = TimeMsg()
    stamp.sec = normalized.sec
    stamp.nanosec = normalized.nanosec

    return stamp


def cov3x3_from_ros(cov: Sequence[float]) -> list[float]:
    """
    Convert a ROS covariance list into a validated 3x3 covariance
    """

    return cov3x3_from_seq(cov)


def vector3_from_ros(msg: Vector3Like) -> list[float]:
    """
    Convert a ROS Vector3-like message into a list
    """

    return vector3_from_xyz(msg.x, msg.y, msg.z)
