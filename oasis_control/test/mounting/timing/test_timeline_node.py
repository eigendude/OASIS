################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for the timeline coordinator."""

from __future__ import annotations

import pytest

from oasis_control.localization.mounting.timing.replay_engine import ReplayEngine
from oasis_control.localization.mounting.timing.replay_engine import ReplayEvent
from oasis_control.localization.mounting.timing.timeline_node import Timeline
from oasis_control.localization.mounting.timing.timeline_node import TimelineError


def test_tick_advances_period() -> None:
    """Ensure tick advances time by the fixed period."""
    timeline: Timeline = Timeline(update_rate_hz=2.0)
    t_ns: int = timeline.tick()
    assert t_ns == timeline.period_ns()


def test_tick_with_explicit_time() -> None:
    """Ensure explicit times are accepted when monotonic."""
    timeline: Timeline = Timeline(update_rate_hz=1.0)
    assert timeline.tick(t_ns=10) == 10
    assert timeline.tick(t_ns=10) == 10
    with pytest.raises(TimelineError):
        timeline.tick(t_ns=9)


def test_tick_callbacks_invoked() -> None:
    """Ensure callbacks are invoked with the current time."""
    timeline: Timeline = Timeline(update_rate_hz=1.0)
    seen: list[int] = []

    def _callback(t_ns: int) -> None:
        seen.append(t_ns)

    timeline.add_tick_callback(_callback)
    timeline.tick()
    timeline.tick()
    assert seen == [1_000_000_000, 2_000_000_000]


def test_drive_replay_ordering() -> None:
    """Ensure replay delivers events before tick callbacks."""
    events: list[ReplayEvent] = [
        ReplayEvent(t_ns=5, topic="a", payload="first"),
        ReplayEvent(t_ns=5, topic="b", payload="second"),
        ReplayEvent(t_ns=8, topic="c", payload="third"),
    ]
    engine: ReplayEngine = ReplayEngine(events)
    timeline: Timeline = Timeline(update_rate_hz=1.0)
    seen: list[str] = []

    def _on_event(event: ReplayEvent) -> None:
        seen.append(f"event:{event.topic}")

    def _on_tick(t_ns: int) -> None:
        seen.append(f"tick:{t_ns}")

    timeline.add_tick_callback(_on_tick)
    timeline.drive_replay(engine, on_event=_on_event)
    assert seen == [
        "event:a",
        "event:b",
        "tick:5",
        "event:c",
        "tick:8",
    ]
