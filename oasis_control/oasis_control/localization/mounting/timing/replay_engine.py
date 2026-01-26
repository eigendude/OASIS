################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Deterministic replay of timestamped events."""

from __future__ import annotations

from dataclasses import dataclass

from oasis_control.localization.mounting.timing.time_base import TimeBase
from oasis_control.localization.mounting.timing.time_base import TimeBaseError


class ReplayError(Exception):
    """Raised when replay operations fail."""


@dataclass(frozen=True)
class ReplayEvent:
    """Timestamped event payload for deterministic replay.

    Attributes:
        t_ns: Event timestamp in nanoseconds, expected to be non-negative
        topic: Non-empty stream identifier for the event
        payload: Opaque event payload, passed through unchanged
    """

    t_ns: int
    topic: str
    payload: object

    def __post_init__(self) -> None:
        """Validate replay event fields."""
        if self.t_ns < 0:
            raise ReplayError("Timestamp must be non-negative")
        if not self.topic:
            raise ReplayError("Topic must be non-empty")


class ReplayEngine:
    """Iterate through events in deterministic time order."""

    def __init__(
        self, events: list[ReplayEvent], *, time_base: TimeBase | None = None
    ) -> None:
        """Create the replay engine from a list of events."""
        self._time_base: TimeBase = time_base or TimeBase(allow_equal=True)
        indexed: list[tuple[int, ReplayEvent]] = list(enumerate(events))
        indexed.sort(key=lambda pair: (pair[1].t_ns, pair[0]))
        self._events: list[ReplayEvent] = [event for _, event in indexed]
        t_prev_ns: int | None = None
        for event in self._events:
            try:
                self._time_base.validate_non_decreasing(t_prev_ns, event.t_ns)
            except TimeBaseError as exc:
                raise ReplayError(str(exc)) from exc
            t_prev_ns = event.t_ns
        self._cursor: int = 0

    def reset(self) -> None:
        """Reset playback to the first event."""
        self._cursor = 0

    def has_next(self) -> bool:
        """Return True if more events remain."""
        return self._cursor < len(self._events)

    def peek_next_time_ns(self) -> int | None:
        """Return the next event time without advancing."""
        if not self.has_next():
            return None
        return self._events[self._cursor].t_ns

    def step(self) -> ReplayEvent:
        """Return the next event, advancing the cursor."""
        if not self.has_next():
            raise ReplayError("No more events")
        event: ReplayEvent = self._events[self._cursor]
        self._cursor += 1
        return event

    def run_until(self, t_ns_inclusive: int) -> list[ReplayEvent]:
        """Return events up to and including the given timestamp."""
        results: list[ReplayEvent] = []
        while self.has_next():
            event: ReplayEvent = self._events[self._cursor]
            if event.t_ns > t_ns_inclusive:
                break
            results.append(event)
            self._cursor += 1
        return results

    def run_all(self) -> list[ReplayEvent]:
        """Return all remaining events."""
        return self.run_until(t_ns_inclusive=2**63 - 1)
