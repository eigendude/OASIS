################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Timing utilities for mounting calibration."""

from __future__ import annotations

import math
from dataclasses import dataclass


class TimeBaseError(Exception):
    """Raised when time conversions or validation fail."""


def sec_to_ns(t_sec: float) -> int:
    """Convert seconds to integer nanoseconds with deterministic rounding.

    Rounds to the nearest integer nanosecond using Python's built-in round
    (ties-to-even) to keep conversion stable across runs.
    """
    if not math.isfinite(t_sec):
        raise TimeBaseError("Seconds must be finite")
    if t_sec < 0.0:
        raise TimeBaseError("Seconds must be non-negative")
    return int(round(t_sec * 1e9))


def ns_to_sec(t_ns: int) -> float:
    """Convert integer nanoseconds to seconds."""
    if t_ns < 0:
        raise TimeBaseError("Nanoseconds must be non-negative")
    return float(t_ns) / 1e9


@dataclass(frozen=True)
class TimeBase:
    """Validate monotonic timestamp sequences for mounting calibration.

    Attributes:
        allow_equal: True to allow repeated timestamps in a sequence
    """

    allow_equal: bool = True

    def validate_non_decreasing(self, t_prev_ns: int | None, t_ns: int) -> None:
        """Validate that timestamps do not decrease."""
        if t_prev_ns is None:
            return
        if self.allow_equal:
            if t_ns < t_prev_ns:
                raise TimeBaseError(
                    "Timestamp must be non-decreasing when allow_equal is True"
                )
            return
        if t_ns <= t_prev_ns:
            raise TimeBaseError(
                "Timestamp must be strictly increasing when allow_equal is False"
            )

    def clamp_monotonic(self, t_prev_ns: int | None, t_ns: int) -> int:
        """Clamp the timestamp to avoid decreases when equality is allowed."""
        if t_prev_ns is None:
            return t_ns
        if not self.allow_equal:
            self.validate_non_decreasing(t_prev_ns, t_ns)
            return t_ns
        if t_ns < t_prev_ns:
            return t_prev_ns
        return t_ns
