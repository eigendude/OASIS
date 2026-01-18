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
import math
from typing import Protocol

from builtin_interfaces.msg import Time as TimeMsg

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

    return AhrsTime(sec=int(stamp.sec), nanosec=int(stamp.nanosec))


def ros_time_from_ahrs(t: AhrsTime) -> TimeMsg:
    """
    Convert an AHRS time into a ROS time stamp
    """

    carry_sec: int = t.nanosec // 1_000_000_000
    nanosec: int = t.nanosec % 1_000_000_000
    sec: int = t.sec + carry_sec

    stamp: TimeMsg = TimeMsg()
    stamp.sec = sec
    stamp.nanosec = nanosec

    return stamp


def cov3x3_from_ros(cov: Sequence[float]) -> list[float]:
    """
    Convert a ROS covariance list into a validated 3x3 covariance
    """

    if len(cov) != 9:
        raise ValueError("Expected 9 covariance entries")

    values: list[float] = [float(value) for value in cov]

    if not all(math.isfinite(value) for value in values):
        raise ValueError("Covariance entries must be finite")

    return values


def vector3_from_ros(msg: Vector3Like) -> list[float]:
    """
    Convert a ROS Vector3-like message into a list
    """

    return [float(msg.x), float(msg.y), float(msg.z)]
