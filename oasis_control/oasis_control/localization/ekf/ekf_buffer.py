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
from oasis_control.localization.ekf.ekf_types import to_ns


class EkfBuffer:
    """
    Fixed-lag buffer that stores events in time order
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._events: list[EkfEvent] = []
        self._timestamps_ns: list[int] = []
        # _latest_time_ns is the max ever inserted, not the max currently in the buffer
        self._latest_time_ns: Optional[int] = None

    def insert_event(self, event: EkfEvent) -> None:
        event_time_ns: int = to_ns(event.t_meas)
        insert_index: int = bisect_right(self._timestamps_ns, event_time_ns)
        self._timestamps_ns.insert(insert_index, event_time_ns)
        self._events.insert(insert_index, event)
        if self._latest_time_ns is None:
            self._latest_time_ns = event_time_ns
        else:
            self._latest_time_ns = max(self._latest_time_ns, event_time_ns)

    def reset(self) -> None:
        self._events = []
        self._timestamps_ns = []
        self._latest_time_ns = None

    def too_old(self, t_meas: EkfTime, *, t_filter_ns: Optional[int]) -> bool:
        if t_filter_ns is None:
            return False

        t_meas_ns: int = to_ns(t_meas)
        cutoff_ns: int = t_filter_ns - self._config.t_buffer_ns
        return t_meas_ns < cutoff_ns

    def evict(self, t_filter_ns: int) -> None:
        cutoff: int = t_filter_ns - self._config.t_buffer_ns
        index: int = bisect_left(self._timestamps_ns, cutoff)
        if index > 0:
            del self._events[:index]
            del self._timestamps_ns[:index]

    def iter_events(self) -> Iterator[EkfEvent]:
        yield from self._events

    def iter_events_from(self, t_start: int) -> Iterator[EkfEvent]:
        start_index: int = bisect_left(self._timestamps_ns, t_start)
        for event in self._events[start_index:]:
            yield event

    def earliest_time(self) -> Optional[int]:
        if not self._timestamps_ns:
            return None
        return self._timestamps_ns[0]

    def latest_time_ns(self) -> Optional[int]:
        return self._latest_time_ns
