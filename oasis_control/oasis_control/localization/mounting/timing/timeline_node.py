################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Deterministic timeline coordinator for mounting calibration."""

from __future__ import annotations

from collections.abc import Callable

from oasis_control.localization.mounting.timing.replay_engine import ReplayEngine
from oasis_control.localization.mounting.timing.replay_engine import ReplayEvent
from oasis_control.localization.mounting.timing.time_base import sec_to_ns


class TimelineError(Exception):
    """Raised when timeline operations fail."""


class Timeline:
    """Advance time deterministically and invoke tick callbacks."""

    def __init__(self, *, update_rate_hz: float) -> None:
        """Initialize the timeline with a fixed update rate."""
        if update_rate_hz <= 0.0:
            raise TimelineError("Update rate must be positive")
        period_ns: int = sec_to_ns(1.0 / update_rate_hz)
        if period_ns <= 0:
            raise TimelineError("Update rate yields non-positive period")
        self._period_ns: int = period_ns
        self._t_ns: int = 0
        self._tick_count: int = 0
        self._callbacks: list[Callable[[int], None]] = []

    def add_tick_callback(self, callback: Callable[[int], None]) -> None:
        """Register a callback invoked on every tick."""
        self._callbacks.append(callback)

    def reset(self) -> None:
        """Reset the timeline to time zero."""
        self._t_ns = 0
        self._tick_count = 0

    def current_time_ns(self) -> int:
        """Return the current time in nanoseconds."""
        return self._t_ns

    def period_ns(self) -> int:
        """Return the fixed period in nanoseconds."""
        return self._period_ns

    def _advance_time(self, t_ns: int) -> None:
        """Advance time with monotonic validation."""
        if t_ns < self._t_ns:
            raise TimelineError("Timeline time must be non-decreasing")
        self._t_ns = t_ns

    def _invoke_callbacks(self) -> None:
        """Invoke tick callbacks once for the current time."""
        for callback in self._callbacks:
            callback(self._t_ns)
        self._tick_count += 1

    def tick(self, *, t_ns: int | None = None) -> int:
        """Advance time by one period or to a specified timestamp."""
        if t_ns is None:
            self._advance_time(self._t_ns + self._period_ns)
        else:
            self._advance_time(t_ns)
        self._invoke_callbacks()
        return self._t_ns

    def drive_replay(
        self,
        engine: ReplayEngine,
        *,
        on_event: Callable[[ReplayEvent], None],
    ) -> None:
        """Drive a replay engine, delivering events before tick callbacks."""
        while engine.has_next():
            next_time_ns: int | None = engine.peek_next_time_ns()
            if next_time_ns is None:
                break
            self._advance_time(next_time_ns)
            events: list[ReplayEvent] = engine.run_until(next_time_ns)
            for event in events:
                if event.t_ns != next_time_ns:
                    raise TimelineError("Replay event time mismatch")
                on_event(event)
            self._invoke_callbacks()
