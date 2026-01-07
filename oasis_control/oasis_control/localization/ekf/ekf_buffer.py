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

from typing import Optional

from oasis_control.localization.ekf.ekf_config import EkfConfig
from oasis_control.localization.ekf.ekf_types import EkfEvent


class EkfBuffer:
    """
    Fixed-lag buffer that stores events in time order
    """

    def __init__(self, config: EkfConfig) -> None:
        self._config: EkfConfig = config
        self._events: list[EkfEvent] = []
        self._latest_time: Optional[float] = None

    def insert_event(self, event: EkfEvent) -> None:
        self._events.append(event)
        self._events.sort(key=lambda item: item.t_meas)
        self._latest_time = event.t_meas

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
        self._events = [event for event in self._events if event.t_meas >= cutoff]
