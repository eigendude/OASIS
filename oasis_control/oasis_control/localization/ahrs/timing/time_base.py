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
        - to_seconds(t_ns)
        - from_seconds(t_sec)
        - validate_monotonic(t_prev_ns, t_next_ns)

    Responsibility:
        Define the timestamp terms used by the AHRS core and provide conversion
        utilities without relying on ROS time types.

    Inputs/outputs:
        - Inputs: raw timestamps or floating-point seconds for conversion.
        - Outputs: canonical integer nanoseconds (t_ns) used by buffers/replay.

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
        - Boundary helpers may convert to/from float seconds for convenience
          only (diagnostics/UI). Float seconds MUST NOT be used for keying,
          ordering, equality, or buffer attachment.

    Determinism and edge cases:
        - Timestamp equality is exact integer equality for keying.
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - Validation should reject negative timestamps and NaN seconds
          inputs.
        - from_seconds(t_sec) and to_seconds(t_ns) are convenience-only
          conversions and are lossy.
        - Float conversions must be explicit and MUST NOT be used for key
          comparisons, ordering, or buffer attachment.

    Equations:
        - No equations; this module defines time terminology.

    Numerical stability notes:
        - Avoid float rounding when converting between units.

    Suggested unit tests:
        - to_seconds/from_seconds round-trip consistency.
        - validate_monotonic rejects decreasing timestamps.
    """

    pass
