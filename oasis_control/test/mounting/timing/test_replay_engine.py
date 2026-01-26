################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the replay engine."""

from __future__ import annotations

import pytest

from oasis_control.localization.mounting.timing.replay_engine import ReplayEngine
from oasis_control.localization.mounting.timing.replay_engine import ReplayError
from oasis_control.localization.mounting.timing.replay_engine import ReplayEvent


def _events() -> list[ReplayEvent]:
    """Create a deterministic list of events for testing."""
    return [
        ReplayEvent(t_ns=5, topic="a", payload=1),
        ReplayEvent(t_ns=3, topic="b", payload=2),
        ReplayEvent(t_ns=3, topic="c", payload=3),
        ReplayEvent(t_ns=8, topic="d", payload=4),
    ]


def test_events_sorted_stable() -> None:
    """Ensure sorting is stable for equal timestamps."""
    engine: ReplayEngine = ReplayEngine(_events())
    ordered: list[ReplayEvent] = engine.run_all()
    assert [event.topic for event in ordered] == ["b", "c", "a", "d"]


def test_step_and_peek() -> None:
    """Ensure stepping and peeking work as expected."""
    engine: ReplayEngine = ReplayEngine(_events())
    first_time: int | None = engine.peek_next_time_ns()
    assert first_time == 3
    event: ReplayEvent = engine.step()
    assert event.topic == "b"
    assert engine.peek_next_time_ns() == 3


def test_run_until() -> None:
    """Ensure run_until returns correct subset."""
    engine: ReplayEngine = ReplayEngine(_events())
    subset: list[ReplayEvent] = engine.run_until(3)
    assert [event.topic for event in subset] == ["b", "c"]
    remaining: list[ReplayEvent] = engine.run_all()
    assert [event.topic for event in remaining] == ["a", "d"]


def test_step_raises_when_empty() -> None:
    """Ensure step raises once exhausted."""
    engine: ReplayEngine = ReplayEngine([])
    with pytest.raises(ReplayError):
        engine.step()
