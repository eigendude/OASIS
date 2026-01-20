################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

class AhrsDiagnostics:
    """Diagnostics payload definitions for the AHRS core.

    Responsibility:
        Document the diagnostics fields produced by buffering and replay
        logic.

    Purpose:
        Provide a structured summary of buffer and replay behavior for logging
        and monitoring.

    Inputs/outputs:
        - Inputs: internal counters and timing state.
        - Outputs: structured diagnostics for integration layers.

    Dependencies:
        - Produced by RingBuffer and ReplayEngine.

    Public API (to be implemented):
        - as_dict()
        - reset_counters()

    Data contract:
        Required fields:
        - buffer_size: current number of nodes in the buffer.
        - t_filter_ns: current filter frontier timestamp in int nanoseconds.
        - drop_count: number of dropped measurements.
        - duplicate_count: number of rejected duplicates.
        - replay_count: number of replay operations.
        - last_replay_from_ns: last replay start in int nanoseconds.
        - rejected_old_count: inserts older than
          (t_filter_ns - t_buffer_ns).
        - rejected_future_count: inserts too far in the future in
          nanoseconds.
        - duplicate_imu_count: rejected duplicate IMU slots.
        - duplicate_mag_count: rejected duplicate mag slots.
        - out_of_order_insert_count: inserts earlier than t_filter_ns.
        - evicted_node_count: number of nodes evicted by horizon.

    Frames and units:
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - Time fields use TimeBase canonical int nanoseconds.

    Determinism and edge cases:
        - Counters increment deterministically for identical input sequences.
        - Counters are monotonic and updated deterministically.
        - Duplicate-slot insertion increments duplicate counters and is
          rejected with no merging or rounding.

    Equations:
        - None; this is a diagnostics container.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - duplicate_count increments on duplicate insertion.
        - replay_count increments when replay occurs.
    """

    pass
