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

from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import from_ns
from oasis_control.localization.ekf.ekf_types import from_seconds
from oasis_control.localization.ekf.ekf_types import to_ns
from oasis_control.localization.ekf.ekf_types import to_seconds


def test_ekf_time_round_trip_seconds() -> None:
    original_ns: int = 123_456_789
    timestamp: EkfTime = from_ns(original_ns)
    seconds: float = to_seconds(timestamp)
    round_trip: EkfTime = from_seconds(seconds)

    assert to_ns(round_trip) == original_ns


def test_ekf_time_from_seconds_rollover() -> None:
    seconds: float = 1.999_999_999_6
    timestamp: EkfTime = from_seconds(seconds)

    assert timestamp.sec == 2
    assert timestamp.nanosec == 0
