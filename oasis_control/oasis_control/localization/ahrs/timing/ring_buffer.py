################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from typing import Dict
from typing import List
from typing import Optional

from oasis_control.localization.ahrs.timing.time_base import TimeBase
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


class RingBuffer:
    """Bounded ring buffer of measurement timeline nodes.

    Purpose:
        Store time-keyed timeline nodes containing state/covariance plus
        measurements and enforce duplicate rejection and eviction policies
        prior to replay.

    Responsibility:
        Provide bounded storage for timeline nodes keyed by t_meas_ns,
        including duplicate rejection, eviction, and diagnostics hooks.

    Inputs/outputs:
        - Inputs: measurement packets with t_meas_ns keys.
        - Outputs: ordered TimelineNode instances containing posterior
          state/cov + measurements.

    Dependencies:
        - Uses TimelineNode instances for storage.
        - Depends on TimeBase for timestamp handling.

    Public API:
        - insert(node)
        - get(t_meas_ns)
        - drop_before(t_cutoff_ns)
        - size()

    Data contract:
        - Nodes are keyed by exact t_meas_ns values.
        - A node may include <=1 IMU packet and <=1 mag packet.
        - Node state/covariance store the posterior after updates at
          t_meas_ns.
        - Eviction drops nodes older than
          (t_filter_ns - t_buffer_ns).

    Frames and units:
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - Times are int nanoseconds as defined by TimeBase.

    Determinism and edge cases:
        - Duplicate same-type at identical t_meas_ns is rejected
          deterministically (no replacement/merge) and diagnosed.
        - Duplicate slot insertion in an existing node is rejected and
          increments diagnostics counters.
        - Inserts older than the buffer horizon are rejected and increment
          diagnostics counters.
        - Out-of-order inserts trigger replay in ReplayEngine.

    Equations:
        - No equations; this module enforces buffer rules.

    Numerical stability notes:
        - None; operations are discrete and deterministic.

    Suggested unit tests:
        - Duplicate IMU at same t_meas_ns is rejected.
        - Old nodes are dropped when older than the buffer horizon.
    """

    def __init__(self, t_buffer_ns: int) -> None:
        if not self._is_int(t_buffer_ns) or t_buffer_ns <= 0:
            raise ValueError("t_buffer_ns must be positive")
        self.t_buffer_ns: int = t_buffer_ns
        self._nodes: Dict[int, TimelineNode] = {}
        self.diagnostics: dict[str, int] = {
            "reject_too_old": 0,
            "duplicate_node": 0,
            "evicted": 0,
        }

    def size(self) -> int:
        """Return the number of nodes stored."""
        return len(self._nodes)

    def get(self, t_meas_ns: int) -> Optional[TimelineNode]:
        """Return the node for the given timestamp."""
        TimeBase.validate_non_negative(t_meas_ns)
        return self._nodes.get(t_meas_ns)

    def drop_before(self, t_cutoff_ns: int) -> int:
        """Drop nodes strictly older than the cutoff time."""
        TimeBase.validate_non_negative(t_cutoff_ns)
        keys_to_drop: List[int] = [
            key for key in self._nodes.keys() if key < t_cutoff_ns
        ]
        key: int
        for key in keys_to_drop:
            del self._nodes[key]
        return len(keys_to_drop)

    def insert(self, node: TimelineNode, *, t_filter_ns: int) -> bool:
        """Insert a node keyed by t_meas_ns and evict old nodes."""
        TimeBase.validate_non_negative(t_filter_ns)
        TimeBase.validate_non_negative(node.t_meas_ns)
        cutoff_ns: int = t_filter_ns - self.t_buffer_ns
        if node.t_meas_ns < cutoff_ns:
            self.diagnostics["reject_too_old"] += 1
            return False
        if node.t_meas_ns in self._nodes:
            self.diagnostics["duplicate_node"] += 1
            return False
        self._nodes[node.t_meas_ns] = node
        evicted: int = self.drop_before(cutoff_ns)
        if evicted:
            self.diagnostics["evicted"] += evicted
        return True

    def nodes_from(self, t_start_ns: int) -> List[TimelineNode]:
        """Return nodes at or after the given timestamp in order."""
        TimeBase.validate_non_negative(t_start_ns)
        keys: List[int] = sorted(key for key in self._nodes.keys() if key >= t_start_ns)
        return [self._nodes[key] for key in keys]

    @staticmethod
    def _is_int(value: object) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)
