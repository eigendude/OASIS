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

from bisect import bisect_right
from collections import deque
from typing import Optional

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent


class EkfBuffer:
    """
    Fixed-lag buffer that stores events in time order
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._events: deque[EkfEvent] = deque()
        # Monotonic max of all inserted timestamps, even after eviction
        self._latest_time: Optional[float] = None

    def insert_event(self, event: EkfEvent) -> None:
        timestamps: list[float] = [item.t_meas for item in self._events]
        insert_index: int = bisect_right(timestamps, event.t_meas)
        self._events.insert(insert_index, event)
        if self._latest_time is None:
            self._latest_time = event.t_meas
        else:
            self._latest_time = max(self._latest_time, event.t_meas)

    def too_old(self, t_meas: float) -> bool:
        if self._latest_time is None:
            return False

        return t_meas < (self._latest_time - self._config.t_buffer_sec)

    def detect_clock_jump(self, t_meas: float) -> bool:
        if self._latest_time is None:
            return False

        dt: float = t_meas - self._latest_time
        return abs(dt) > self._config.dt_clock_jump_max

    def evict(self, t_filter: float) -> None:
        cutoff: float = t_filter - self._config.t_buffer_sec
        while self._events and self._events[0].t_meas < cutoff:
            self._events.popleft()
