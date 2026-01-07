################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Fixed-lag EKF event buffer
"""

from __future__ import annotations

from bisect import bisect_left
from bisect import bisect_right
from typing import Iterator
from typing import Optional

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent
from oasis_control.localization.ekf.ekf_types import EkfTime
from oasis_control.localization.ekf.ekf_types import to_seconds


class EkfBuffer:
    """
    Fixed-lag buffer that stores events in time order
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._events: list[EkfEvent] = []
        self._timestamps: list[float] = []
        # _latest_time is the max ever inserted, not the max currently in the buffer
        self._latest_time: Optional[float] = None

    def insert_event(self, event: EkfEvent) -> None:
        event_time_s: float = to_seconds(event.t_meas)
        insert_index: int = bisect_right(self._timestamps, event_time_s)
        self._timestamps.insert(insert_index, event_time_s)
        self._events.insert(insert_index, event)
        if self._latest_time is None:
            self._latest_time = event_time_s
        else:
            self._latest_time = max(self._latest_time, event_time_s)

    def reset(self) -> None:
        self._events = []
        self._timestamps = []
        self._latest_time = None

    def too_old(self, t_meas: EkfTime) -> bool:
        if self._latest_time is None:
            return False

        t_meas_s: float = to_seconds(t_meas)
        return t_meas_s < (self._latest_time - self._config.t_buffer_sec)

    def detect_clock_jump(self, t_meas: EkfTime) -> bool:
        if self._latest_time is None:
            return False

        t_meas_s: float = to_seconds(t_meas)
        dt: float = t_meas_s - self._latest_time
        return abs(dt) > self._config.dt_clock_jump_max

    def evict(self, t_filter: EkfTime) -> None:
        t_filter_s: float = to_seconds(t_filter)
        cutoff: float = t_filter_s - self._config.t_buffer_sec
        index: int = 0
        while index < len(self._events) and self._timestamps[index] < cutoff:
            index += 1
        if index > 0:
            del self._events[:index]
            del self._timestamps[:index]

    def iter_events(self) -> Iterator[EkfEvent]:
        yield from self._events

    def iter_events_from(self, t_start: float) -> Iterator[EkfEvent]:
        start_index: int = bisect_left(self._timestamps, t_start)
        for event in self._events[start_index:]:
            yield event

    def earliest_time(self) -> Optional[float]:
        if not self._timestamps:
            return None
        return self._timestamps[0]

    def latest_time(self) -> Optional[float]:
        return self._latest_time
