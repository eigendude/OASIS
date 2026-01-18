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
Time-ordered event buffer for AHRS localization
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
from typing import Iterator

from oasis_control.localization.ahrs.ahrs_conversions import seconds_from_ahrs
from oasis_control.localization.ahrs.ahrs_types import AhrsEvent
from oasis_control.localization.ahrs.ahrs_types import AhrsTime


@dataclass(frozen=False)
class AhrsTimeNode:
    """
    Time node containing events at a single timestamp

    Fields:
        t: Timestamp for this node
        events: Events recorded at the timestamp
    """

    t: AhrsTime
    events: list[AhrsEvent]


class AhrsTimeBuffer:
    """
    Time-ordered buffer of AHRS events
    """

    def __init__(self) -> None:
        self._nodes: list[AhrsTimeNode] = []

    def get_or_create_node(self, t: AhrsTime) -> AhrsTimeNode:
        """
        Retrieve or create the node for the requested time
        """

        index: int = self._index_for_time(t)
        if index < len(self._nodes) and self._same_time(self._nodes[index].t, t):
            return self._nodes[index]

        node: AhrsTimeNode = AhrsTimeNode(t=t, events=[])
        self._nodes.insert(index, node)
        return node

    def insert_event(self, event: AhrsEvent) -> None:
        """
        Insert an event into the buffer at its timestamp
        """

        node: AhrsTimeNode = self.get_or_create_node(event.t_meas)
        node.events.append(event)

    def evict_older_than(self, t_min: AhrsTime) -> int:
        """
        Evict nodes older than the minimum time
        """

        index: int = self._index_for_time(t_min)
        if index == 0:
            return 0

        del self._nodes[:index]
        return index

    def node_count(self) -> int:
        """
        Return the number of buffered time nodes
        """

        return len(self._nodes)

    def span_sec(self) -> float:
        """
        Return the span in seconds between oldest and newest nodes
        """

        if len(self._nodes) < 2:
            return 0.0

        oldest: AhrsTime = self._nodes[0].t
        newest: AhrsTime = self._nodes[-1].t
        return seconds_from_ahrs(newest) - seconds_from_ahrs(oldest)

    def iter_nodes(self) -> Iterator[AhrsTimeNode]:
        """
        Iterate over buffered nodes in time order
        """

        return iter(self._nodes)

    def iter_nodes_from(self, t: AhrsTime) -> Iterator[AhrsTimeNode]:
        """
        Iterate over buffered nodes starting at a time
        """

        index: int = self._index_for_time(t)
        return iter(self._nodes[index:])

    def _index_for_time(self, t: AhrsTime) -> int:
        keys: list[tuple[int, int]] = [self._time_key(node.t) for node in self._nodes]
        return bisect_left(keys, self._time_key(t))

    def _same_time(self, left: AhrsTime, right: AhrsTime) -> bool:
        return self._time_key(left) == self._time_key(right)

    def _time_key(self, t: AhrsTime) -> tuple[int, int]:
        return (t.sec, t.nanosec)
