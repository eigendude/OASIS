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
Conversion helpers between core AHRS types
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from oasis_control.localization.ahrs.ahrs_types import AhrsTime


def ahrs_time_from_pair(sec: int, nanosec: int) -> AhrsTime:
    """
    Build an AHRS time from a seconds/nanoseconds pair
    """

    return normalize_ahrs_time(AhrsTime(sec=int(sec), nanosec=int(nanosec)))


def normalize_ahrs_time(t: AhrsTime) -> AhrsTime:
    """
    Normalize nanoseconds so they always live in [0, 1e9)
    """

    carry_sec: int
    nanosec: int
    carry_sec, nanosec = divmod(t.nanosec, 1_000_000_000)
    sec: int = t.sec + carry_sec

    return AhrsTime(sec=sec, nanosec=nanosec)


def cov3x3_from_seq(cov: Sequence[float]) -> list[float]:
    """
    Validate and convert a covariance sequence into a 3x3 covariance
    """

    if len(cov) != 9:
        raise ValueError("Expected 9 covariance entries")

    values: list[float] = [float(value) for value in cov]

    if not all(math.isfinite(value) for value in values):
        raise ValueError("Covariance entries must be finite")

    return values


def vector3_from_xyz(x: float, y: float, z: float) -> list[float]:
    """
    Convert XYZ components into a vector list
    """

    return [float(x), float(y), float(z)]


def seconds_from_ahrs(t: AhrsTime) -> float:
    """
    Convert an AHRS timestamp into seconds
    """

    return float(t.sec) + float(t.nanosec) * 1e-9
