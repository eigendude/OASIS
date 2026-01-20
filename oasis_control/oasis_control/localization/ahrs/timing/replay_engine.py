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

from dataclasses import dataclass
from typing import Callable
from typing import Optional
from typing import Protocol

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.timing.ring_buffer import RingBuffer
from oasis_control.localization.ahrs.timing.time_base import TimeBase
from oasis_control.localization.ahrs.timing.timeline_node import TimelineNode


class ReplayEkf(Protocol):
    """Protocol describing the EKF API needed by ReplayEngine."""

    def propagate_to(self, t_ns: int) -> None:
        """Propagate the filter state to t_ns."""

    def update_imu(self, imu_packet: ImuPacket) -> None:
        """Apply the IMU measurement update."""

    def update_mag(self, mag_packet: MagPacket) -> None:
        """Apply the magnetometer measurement update."""

    def update_stationary(self, stationary_packet: StationaryPacket) -> None:
        """Apply the stationary pseudo-update."""

    def get_state(self) -> AhrsState:
        """Return the current filter state."""

    def get_covariance(self) -> AhrsCovariance:
        """Return the current filter covariance."""


@dataclass(slots=True)
class ReplayEngine:
    """Replay logic for time-ordered AHRS measurements.

    Purpose:
        Deterministically replay buffered measurements when data arrives out
        of order and manage the filter frontier time.

    Responsibility:
        Provide deterministic replay of buffered measurements when
        out-of-order inserts occur and control publication at the filter
        frontier.

    Inputs/outputs:
        - Inputs: TimelineNode instances keyed by t_meas_ns.
        - Outputs: updated filter state and diagnostics counts.

    Dependencies:
        - Uses RingBuffer and TimelineNode for storage.
        - Interfaces with AhrsEkf for updates.

    Replay algorithm (spec 7.2/7.3):
        1) Validate timestamp against TimeBase rules.
        2) Reject if older than t_filter_ns - t_buffer_ns.
        3) Attach to node (create if missing); reject duplicate slot
           inserts.
        4) If inserted in the past, replay node-by-node forward to frontier.
        5) Publish only when frontier advances; out-of-order updates do not
           republish.

    Public API (to be implemented):
        - insert_imu(imu_packet)
        - insert_mag(mag_packet)
        - insert_stationary(stationary_packet)
        - replay_from(t_meas_ns)
        - frontier_time()

    Data contract:
        - Nodes are keyed by t_meas_ns and contain <=1 measurement per type.
        - Duplicate same-type at same t_meas_ns must be rejected
          deterministically (no replacement/merge) and diagnosed.
        - A node may contain an IMU packet and mag packet together when
          they share the same t_meas_ns.
        - A node may contain a stationary packet keyed by the same
          t_meas_ns.
        - Node state/covariance store the posterior after updates at
          t_meas_ns.
        - Replay recomputes posterior state/covariance node-by-node in time
          order.

    Frames and units:
        - All timestamps follow TimeBase (int nanoseconds).
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.

    Determinism and edge cases:
        - Out-of-order insert => replay forward to frontier.
        - Out-of-order insert does not trigger immediate publish.
        - Publish only when frontier advances.
        - Duplicate inserts are rejected without modifying the node.
        - Exact t_meas_ns match determines node attachment.
        - No epsilon merging, rounding, or "close enough" matching is
          allowed.
        - Update order within a node:
            1) priors once
            2) gyro update
            3) accel update
            4) mag update
            5) if stationary packet present and is_stationary:
               5a) no-turn update (if enabled)
               5b) ZUPT update

    Equations:
        - No equations; procedural replay rules are specified above.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - Out-of-order insert triggers replay.
        - Duplicate measurements are rejected with diagnostics.
    """

    ring_buffer: RingBuffer
    _t_filter_ns: int
    _t_buffer_ns: int
    _ekf: ReplayEkf
    _publish_callback: Optional[Callable[[int], None]]
    diagnostics: dict[str, int]

    def __init__(
        self,
        *,
        t_buffer_ns: int,
        ekf: ReplayEkf,
        publish_callback: Optional[Callable[[int], None]] = None,
    ) -> None:
        if not self._is_int(t_buffer_ns) or t_buffer_ns <= 0:
            raise ValueError("t_buffer_ns must be positive")
        self.ring_buffer = RingBuffer(t_buffer_ns)
        self._t_filter_ns = 0
        self._t_buffer_ns = t_buffer_ns
        self._ekf = ekf
        self._publish_callback = publish_callback
        self.diagnostics = {
            "reject_too_old": 0,
            "duplicate_imu": 0,
            "duplicate_mag": 0,
            "duplicate_stationary": 0,
        }

    def frontier_time(self) -> int:
        """Return the current filter frontier time."""
        return self._t_filter_ns

    def insert_imu(self, packet: ImuPacket) -> bool:
        """Insert an IMU packet and replay deterministically if needed."""
        return self._insert_packet(packet, "imu")

    def insert_mag(self, packet: MagPacket) -> bool:
        """Insert a magnetometer packet and replay deterministically."""
        return self._insert_packet(packet, "mag")

    def insert_stationary(self, packet: StationaryPacket) -> bool:
        """Insert a stationary packet and replay deterministically."""
        return self._insert_packet(packet, "stationary")

    def replay_from(self, t_start_ns: int) -> None:
        """Replay buffered nodes from t_start_ns in time order."""
        TimeBase.validate_non_negative(t_start_ns)
        ordered_items: list[tuple[int, TimelineNode]] = [
            (t_ns, node)
            for t_ns, node in self.ring_buffer._nodes.items()
            if t_ns >= t_start_ns
        ]
        if not ordered_items:
            return
        for t_ns, node in ordered_items:
            self._ekf.propagate_to(t_ns)
            if node.imu_packet is not None:
                self._ekf.update_imu(node.imu_packet)
            if node.mag_packet is not None:
                self._ekf.update_mag(node.mag_packet)
            if (
                node.stationary_packet is not None
                and node.stationary_packet.is_stationary
            ):
                self._ekf.update_stationary(node.stationary_packet)
            node.state = self._ekf.get_state()
            node.covariance = self._ekf.get_covariance()
        last_time: int = ordered_items[-1][0]
        if last_time > self._t_filter_ns:
            self._t_filter_ns = last_time

    def _insert_packet(self, packet: object, packet_type: str) -> bool:
        t_meas_ns: int = self._packet_time(packet)
        TimeBase.validate_non_negative(t_meas_ns)
        cutoff_ns: int = self._t_filter_ns - self._t_buffer_ns
        if t_meas_ns < cutoff_ns:
            self.diagnostics["reject_too_old"] += 1
            return False
        node: Optional[TimelineNode] = self.ring_buffer.get(t_meas_ns)
        if node is None:
            node = TimelineNode(t_meas_ns)
            if not self.ring_buffer.insert(node, t_filter_ns=self._t_filter_ns):
                return False
        inserted: bool
        if packet_type == "imu":
            inserted = node.insert_imu(packet)  # type: ignore[arg-type]
            if not inserted:
                self.diagnostics["duplicate_imu"] += 1
        elif packet_type == "mag":
            inserted = node.insert_mag(packet)  # type: ignore[arg-type]
            if not inserted:
                self.diagnostics["duplicate_mag"] += 1
        elif packet_type == "stationary":
            inserted = node.insert_stationary(packet)  # type: ignore[arg-type]
            if not inserted:
                self.diagnostics["duplicate_stationary"] += 1
        else:
            raise ValueError("packet_type must be imu, mag, or stationary")
        previous_frontier: int = self._t_filter_ns
        self.replay_from(t_meas_ns)
        if self._t_filter_ns > previous_frontier and self._publish_callback:
            self._publish_callback(self._t_filter_ns)
        return inserted

    @staticmethod
    def _packet_time(packet: object) -> int:
        if not hasattr(packet, "t_meas_ns"):
            raise ValueError("packet must have t_meas_ns")
        t_meas_ns: object = getattr(packet, "t_meas_ns")
        if not isinstance(t_meas_ns, int) or isinstance(t_meas_ns, bool):
            raise ValueError("t_meas_ns must be non-negative")
        return t_meas_ns

    @staticmethod
    def _is_int(value: object) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)
