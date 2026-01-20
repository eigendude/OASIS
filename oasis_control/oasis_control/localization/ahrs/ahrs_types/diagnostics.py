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


@dataclass(frozen=True, slots=True)
class AhrsDiagnostics:
    """Diagnostics payload definitions for the AHRS core.

    Data contract:
        buffer_size:
            Current number of nodes in the buffer for monitoring occupancy
        t_filter_ns:
            Current filter frontier timestamp in int nanoseconds
        drop_count:
            Count of dropped measurements since the last reset
        duplicate_count:
            Count of rejected duplicates since the last reset
        replay_count:
            Count of replay operations since the last reset
        last_replay_from_ns:
            Timestamp in int nanoseconds of the most recent replay start
        rejected_old_count:
            Count of inserts older than the buffer horizon since reset
        rejected_future_count:
            Count of inserts too far in the future since reset
        duplicate_imu_count:
            Count of rejected duplicate IMU slots since reset
        duplicate_mag_count:
            Count of rejected duplicate magnetometer slots since reset
        out_of_order_insert_count:
            Count of inserts earlier than t_filter_ns since reset
        evicted_node_count:
            Count of nodes evicted by the buffer horizon since reset

    Determinism and edge cases:
        - Counters are monotonic and deterministic for identical sequences
        - All timestamps are int nanoseconds and compared exactly
    """

    buffer_size: int
    t_filter_ns: int
    drop_count: int
    duplicate_count: int
    replay_count: int
    last_replay_from_ns: int
    rejected_old_count: int
    rejected_future_count: int
    duplicate_imu_count: int
    duplicate_mag_count: int
    out_of_order_insert_count: int
    evicted_node_count: int

    def validate(self) -> None:
        """Validate counters and timestamps and raise ValueError on failure."""
        if not self._is_int(self.buffer_size):
            raise ValueError("buffer_size must be int")
        if not self._is_int(self.t_filter_ns):
            raise ValueError("t_filter_ns must be int nanoseconds")
        if not self._is_int(self.last_replay_from_ns):
            raise ValueError("last_replay_from_ns must be int nanoseconds")
        for name, value in (
            ("drop_count", self.drop_count),
            ("duplicate_count", self.duplicate_count),
            ("replay_count", self.replay_count),
            ("rejected_old_count", self.rejected_old_count),
            ("rejected_future_count", self.rejected_future_count),
            ("duplicate_imu_count", self.duplicate_imu_count),
            ("duplicate_mag_count", self.duplicate_mag_count),
            ("out_of_order_insert_count", self.out_of_order_insert_count),
            ("evicted_node_count", self.evicted_node_count),
        ):
            if not self._is_int(value):
                raise ValueError(f"{name} must be int")
            if value < 0:
                raise ValueError(f"{name} must be non-negative")

    def reset_counters(self) -> "AhrsDiagnostics":
        """Return a new instance with counters reset and state preserved."""
        return AhrsDiagnostics(
            buffer_size=self.buffer_size,
            t_filter_ns=self.t_filter_ns,
            drop_count=0,
            duplicate_count=0,
            replay_count=0,
            last_replay_from_ns=self.last_replay_from_ns,
            rejected_old_count=0,
            rejected_future_count=0,
            duplicate_imu_count=0,
            duplicate_mag_count=0,
            out_of_order_insert_count=0,
            evicted_node_count=0,
        )

    def as_dict(self) -> dict[str, int]:
        """Return a JSON-serializable dict representation."""
        return {
            "buffer_size": self.buffer_size,
            "t_filter_ns": self.t_filter_ns,
            "drop_count": self.drop_count,
            "duplicate_count": self.duplicate_count,
            "replay_count": self.replay_count,
            "last_replay_from_ns": self.last_replay_from_ns,
            "rejected_old_count": self.rejected_old_count,
            "rejected_future_count": self.rejected_future_count,
            "duplicate_imu_count": self.duplicate_imu_count,
            "duplicate_mag_count": self.duplicate_mag_count,
            "out_of_order_insert_count": self.out_of_order_insert_count,
            "evicted_node_count": self.evicted_node_count,
        }

    @staticmethod
    def _is_int(value: object) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)
