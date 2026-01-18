################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

import math

from builtin_interfaces.msg import Time as TimeMsg
import pytest

from oasis_control.localization.ahrs.ahrs_conversions import ahrs_time_from_ros
from oasis_control.localization.ahrs.ahrs_conversions import cov3x3_from_ros
from oasis_control.localization.ahrs.ahrs_conversions import ros_time_from_ahrs
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


def test_cov3x3_from_ros_rejects_length() -> None:
    with pytest.raises(ValueError):
        cov3x3_from_ros([0.0] * 8)


def test_cov3x3_from_ros_rejects_nan() -> None:
    cov: list[float] = [0.0] * 8 + [math.nan]

    with pytest.raises(ValueError):
        cov3x3_from_ros(cov)


def test_ahrs_time_round_trip() -> None:
    stamp: TimeMsg = TimeMsg()
    stamp.sec = 123
    stamp.nanosec = 456_789_123

    ahrs_time: AhrsTime = ahrs_time_from_ros(stamp)
    round_trip: TimeMsg = ros_time_from_ahrs(ahrs_time)

    assert round_trip.sec == stamp.sec
    assert round_trip.nanosec == stamp.nanosec
