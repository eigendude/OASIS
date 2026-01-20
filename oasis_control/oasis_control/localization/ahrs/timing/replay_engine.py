################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Replay engine for out-of-order AHRS measurements.

Responsibility:
    Provide deterministic replay of buffered measurements when out-of-order
    inserts occur and control publication at the filter frontier.

Inputs/outputs:
    - Inputs: Timeline nodes keyed by t_meas_ns.
    - Outputs: updated filter state and diagnostics counts.

Dependencies:
    - Uses RingBuffer and Timeline for storage.
    - Interfaces with AhrsEkf for updates.

Determinism:
    - Out-of-order insert triggers replay forward to the frontier.
    - Publish only when the frontier advances.
    - Duplicate-slot insertion is rejected without modification.
"""


class ReplayEngine:
    """Replay logic for time-ordered AHRS measurements.

    Purpose:
        Deterministically replay buffered measurements when data arrives out
        of order and manage the filter frontier time.

    Replay algorithm (spec 7.2/7.3):
        1) Validate timestamp against TimeBase rules.
        2) Reject if older than t_filter_ns - T_buffer_sec * 1e9.
        3) Attach to node (create if missing); reject duplicate slot
           inserts.
        4) If inserted in the past, replay node-by-node forward to frontier.
        5) Publish only when frontier advances; out-of-order updates do not
           republish.

    Public API (to be implemented):
        - insert_imu(imu_packet)
        - insert_mag(mag_packet)
        - replay_from(t_meas_ns)
        - frontier_time()

    Data contract:
        - Nodes are keyed by t_meas_ns and contain <=1 measurement per type.
        - Duplicate same-type at same t_meas_ns must be rejected and
          diagnosed.
        - Replay recomputes mean state/covariance node-by-node in time order.

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

    Equations:
        - No equations; procedural replay rules are specified above.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - Out-of-order insert triggers replay.
        - Duplicate measurements are rejected with diagnostics.
    """

    pass
