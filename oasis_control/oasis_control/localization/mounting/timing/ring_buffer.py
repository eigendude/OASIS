################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Deterministic ring buffer for timestamped samples."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Generic
from typing import TypeVar

from oasis_control.localization.mounting.timing.time_base import TimeBase
from oasis_control.localization.mounting.timing.time_base import TimeBaseError


T = TypeVar("T")


class RingBufferError(Exception):
    """Raised when ring buffer operations fail."""


@dataclass(frozen=True)
class Timestamped(Generic[T]):
    """Container for timestamped samples.

    Attributes:
        t_ns: Measurement timestamp in nanoseconds, expected to be non-negative
            and monotonic within a buffer
        value: Sample payload stored alongside the timestamp
    """

    t_ns: int
    value: T

    def __post_init__(self) -> None:
        """Validate that timestamps are non-negative."""
        if self.t_ns < 0:
            raise RingBufferError("Timestamp must be non-negative")


class TimestampRingBuffer(Generic[T]):
    """Store the most recent timestamped samples in deterministic order."""

    def __init__(self, *, capacity: int, time_base: TimeBase | None = None) -> None:
        """Initialize the ring buffer."""
        if capacity <= 0:
            raise RingBufferError("Capacity must be positive")
        self._capacity: int = capacity
        self._time_base: TimeBase = time_base or TimeBase(allow_equal=True)
        self._items: list[Timestamped[T]] = []

    def __len__(self) -> int:
        """Return the number of items stored."""
        return len(self._items)

    def is_empty(self) -> bool:
        """Return True if the buffer is empty."""
        return not self._items

    def clear(self) -> None:
        """Remove all items from the buffer."""
        self._items.clear()

    def push(self, t_ns: int, value: T) -> None:
        """Append a timestamped value, enforcing monotonic time."""
        t_prev_ns: int | None = None
        if self._items:
            t_prev_ns = self._items[-1].t_ns
        try:
            self._time_base.validate_non_decreasing(t_prev_ns, t_ns)
        except TimeBaseError as exc:
            raise RingBufferError(str(exc)) from exc
        item: Timestamped[T] = Timestamped(t_ns=t_ns, value=value)
        self._items.append(item)
        if len(self._items) > self._capacity:
            del self._items[0]

    def latest(self) -> Timestamped[T] | None:
        """Return the newest item, if any."""
        if not self._items:
            return None
        return self._items[-1]

    def earliest(self) -> Timestamped[T] | None:
        """Return the oldest item, if any."""
        if not self._items:
            return None
        return self._items[0]

    def iter_time_order(self) -> list[Timestamped[T]]:
        """Return items in increasing time order."""
        return list(self._items)

    def pop_older_than(self, t_ns_exclusive: int) -> list[Timestamped[T]]:
        """Remove and return all items older than the given timestamp."""
        removed: list[Timestamped[T]] = [
            item for item in self._items if item.t_ns < t_ns_exclusive
        ]
        self._items = [item for item in self._items if item.t_ns >= t_ns_exclusive]
        return removed

    def find_exact(self, t_ns: int) -> list[Timestamped[T]]:
        """Return all items with a matching timestamp."""
        return [item for item in self._items if item.t_ns == t_ns]

    def find_latest_at_or_before(self, t_ns: int) -> Timestamped[T] | None:
        """Return the newest item at or before the given timestamp."""
        for item in reversed(self._items):
            if item.t_ns <= t_ns:
                return item
        return None
