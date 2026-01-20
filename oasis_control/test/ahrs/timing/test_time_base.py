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

import pytest

from oasis_control.localization.ahrs.timing.time_base import TimeBase


def test_validate_non_negative_rejects_negative() -> None:
    with pytest.raises(ValueError, match="t_ns must be non-negative"):
        TimeBase.validate_non_negative(-1)


def test_validate_non_negative_rejects_non_int() -> None:
    with pytest.raises(ValueError, match="t_ns must be non-negative"):
        TimeBase.validate_non_negative(True)


def test_validate_monotonic_rejects_decreasing() -> None:
    with pytest.raises(ValueError, match="timestamps must be monotonic"):
        TimeBase.validate_monotonic(10, 9)


def test_validate_monotonic_rejects_negative() -> None:
    with pytest.raises(ValueError, match="t_ns must be non-negative"):
        TimeBase.validate_monotonic(-1, 0)


def test_stamp_to_ns_converts() -> None:
    assert TimeBase.stamp_to_ns(1, 2) == 1_000_000_002


def test_stamp_to_ns_rejects_sec() -> None:
    with pytest.raises(ValueError, match="sec must be non-negative"):
        TimeBase.stamp_to_ns(-1, 0)


def test_stamp_to_ns_rejects_nanosec() -> None:
    with pytest.raises(ValueError, match="nanosec must be in \\[0, 1_000_000_000\\)"):
        TimeBase.stamp_to_ns(0, 1_000_000_000)
