################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for timestamp ring buffer."""

from __future__ import annotations

import pytest

from oasis_control.localization.mounting.timing.ring_buffer import RingBufferError
from oasis_control.localization.mounting.timing.ring_buffer import Timestamped
from oasis_control.localization.mounting.timing.ring_buffer import TimestampRingBuffer


def test_capacity_discards_oldest() -> None:
    """Ensure the buffer retains only the newest samples."""
    buffer: TimestampRingBuffer[str] = TimestampRingBuffer(capacity=3)
    buffer.push(1, "a")
    buffer.push(2, "b")
    buffer.push(3, "c")
    buffer.push(4, "d")
    items: list[Timestamped[str]] = buffer.iter_time_order()
    assert [item.value for item in items] == ["b", "c", "d"]


def test_non_decreasing_enforced() -> None:
    """Ensure decreasing timestamps are rejected."""
    buffer: TimestampRingBuffer[int] = TimestampRingBuffer(capacity=2)
    buffer.push(10, 1)
    with pytest.raises(RingBufferError):
        buffer.push(9, 2)


def test_equal_timestamp_insertion() -> None:
    """Ensure equal timestamps are allowed and stable."""
    buffer: TimestampRingBuffer[str] = TimestampRingBuffer(capacity=5)
    buffer.push(5, "a")
    buffer.push(5, "b")
    matches: list[Timestamped[str]] = buffer.find_exact(5)
    assert [item.value for item in matches] == ["a", "b"]


def test_iter_time_order_is_stable() -> None:
    """Ensure items remain in insertion order for equal timestamps."""
    buffer: TimestampRingBuffer[str] = TimestampRingBuffer(capacity=5)
    buffer.push(1, "a")
    buffer.push(2, "b")
    buffer.push(2, "c")
    buffer.push(3, "d")
    items: list[Timestamped[str]] = buffer.iter_time_order()
    assert [item.value for item in items] == ["a", "b", "c", "d"]


def test_pop_older_than() -> None:
    """Ensure pop_older_than removes the expected subset."""
    buffer: TimestampRingBuffer[str] = TimestampRingBuffer(capacity=5)
    buffer.push(1, "a")
    buffer.push(2, "b")
    buffer.push(3, "c")
    removed: list[Timestamped[str]] = buffer.pop_older_than(3)
    assert [item.value for item in removed] == ["a", "b"]
    remaining: list[Timestamped[str]] = buffer.iter_time_order()
    assert [item.value for item in remaining] == ["c"]


def test_find_latest_at_or_before() -> None:
    """Ensure latest lookup returns the expected item."""
    buffer: TimestampRingBuffer[str] = TimestampRingBuffer(capacity=5)
    buffer.push(1, "a")
    buffer.push(3, "b")
    buffer.push(5, "c")
    item: Timestamped[str] | None = buffer.find_latest_at_or_before(4)
    assert item is not None
    assert item.value == "b"
