################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class TimeBase:
    """Time base helpers for AHRS buffering and replay logic.

    Purpose:
        Provide a consistent definition of measurement timestamps and the
        filter frontier time used by buffering and replay.

    Public API (to be implemented):
        - validate_monotonic(t_prev_ns, t_next_ns)
        - validate_non_negative(t_ns)

    Responsibility:
        Define the timestamp terms used by the AHRS core and provide conversion
        utilities without relying on ROS time types.

    Inputs/outputs:
        - Inputs: raw timestamps in integer nanoseconds.
        - Outputs: validated integer nanoseconds (t_ns) used by buffers/replay.

    Dependencies:
        - Used by RingBuffer, TimelineNode, and ReplayEngine.

    Data contract:
        - t_meas_ns: measurement timestamp in integer nanoseconds.
        - t_now_ns: current wall-clock timestamp (provided externally).
        - t_filter_ns: filter frontier timestamp after processing.

    Frames and units:
        - Canonical internal time is int nanoseconds (t_ns) sourced from ROS
          header stamps (sec, nsec).
        - Canonical formula:
            t_meas_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
          with nanosec in [0, 1e9).
        - No float conversions exist in the core; any UI or diagnostics
          conversions must occur outside the core.

    Determinism and edge cases:
        - Timestamp equality is exact integer equality for keying.
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - Validation should reject negative timestamps.

    Equations:
        - No equations; this module defines time terminology.

    Numerical stability notes:
        - Avoid float rounding when converting between units.

    Suggested unit tests:
        - stamp-to-ns formula uses sec * 1_000_000_000 + nanosec.
        - validate_monotonic rejects negative or decreasing timestamps.
    """

    pass
