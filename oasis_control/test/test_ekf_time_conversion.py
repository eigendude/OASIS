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
from oasis_control.localization.ekf.ekf_types import to_ns


def test_ekf_time_round_trip_ns() -> None:
    original_ns: int = 123_456_789
    timestamp: EkfTime = from_ns(original_ns)
    round_trip_ns: int = to_ns(timestamp)

    assert round_trip_ns == original_ns


def test_ekf_time_from_ns_rollover() -> None:
    original_ns: int = 2_000_000_000
    timestamp: EkfTime = from_ns(original_ns)

    assert timestamp.sec == 2
    assert timestamp.nanosec == 0
