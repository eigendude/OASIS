################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for time base helpers."""

from __future__ import annotations

import math

import pytest

from oasis_control.localization.mounting.timing.time_base import TimeBase
from oasis_control.localization.mounting.timing.time_base import TimeBaseError
from oasis_control.localization.mounting.timing.time_base import ns_to_sec
from oasis_control.localization.mounting.timing.time_base import sec_to_ns


def test_sec_to_ns_and_back() -> None:
    """Ensure round-trip conversion is stable."""
    t_sec: float = 1.25
    t_ns: int = sec_to_ns(t_sec)
    assert t_ns == int(round(1.25e9))
    t_sec_back: float = ns_to_sec(t_ns)
    assert t_sec_back == pytest.approx(t_sec)


def test_sec_to_ns_rejects_invalid() -> None:
    """Ensure invalid seconds inputs are rejected."""
    with pytest.raises(TimeBaseError):
        sec_to_ns(-1.0)
    with pytest.raises(TimeBaseError):
        sec_to_ns(math.inf)
    with pytest.raises(TimeBaseError):
        sec_to_ns(math.nan)


def test_ns_to_sec_rejects_negative() -> None:
    """Ensure invalid nanoseconds inputs are rejected."""
    with pytest.raises(TimeBaseError):
        ns_to_sec(-5)


def test_validate_non_decreasing_allows_equal() -> None:
    """Ensure equality is allowed by default."""
    base: TimeBase = TimeBase()
    base.validate_non_decreasing(5, 5)
    with pytest.raises(TimeBaseError):
        base.validate_non_decreasing(5, 4)


def test_validate_non_decreasing_strict() -> None:
    """Ensure strict monotonic validation rejects equality."""
    base: TimeBase = TimeBase(allow_equal=False)
    with pytest.raises(TimeBaseError):
        base.validate_non_decreasing(5, 5)
    base.validate_non_decreasing(5, 6)
