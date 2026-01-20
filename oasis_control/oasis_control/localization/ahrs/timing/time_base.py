################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Time base definitions for AHRS buffering and replay.

Responsibility:
    Define the timestamp terms used by the AHRS core and provide conversion
    utilities without relying on ROS time types.

Inputs/outputs:
    - Inputs: raw timestamps or floating-point seconds.
    - Outputs: canonical integer nanoseconds (t_ns) used by buffers/replay.

Dependencies:
    - Used by RingBuffer, Timeline, and ReplayEngine.

Determinism:
    Canonical internal representation is int nanoseconds (t_ns).
    Equality for keying uses exact integer equality with no epsilon.
"""


class TimeBase:
    """Time base helpers for AHRS buffering and replay logic.

    Purpose:
        Provide a consistent definition of measurement timestamps and the
        filter frontier time used by buffering and replay.

    Public API (to be implemented):
        - to_seconds(t)
        - from_seconds(t_sec)
        - validate_monotonic(t_prev, t_next)

    Data contract:
        - t_meas_ns: measurement timestamp in integer nanoseconds.
        - t_now_ns: current wall-clock timestamp (provided externally).
        - t_filter_ns: filter frontier timestamp after processing.

    Frames and units:
        - Canonical storage is int nanoseconds (t_ns).
        - Boundary helpers may convert to/from float seconds for convenience.

    Determinism and edge cases:
        - Timestamp equality is exact integer equality for keying.
        - Validation should reject negative or NaN timestamps.
        - Float conversions must be explicit and loss-aware.

    Equations:
        - No equations; this module defines time terminology.

    Numerical stability notes:
        - Avoid float rounding when converting between units.

    Suggested unit tests:
        - to_seconds/from_seconds round-trip consistency.
        - validate_monotonic rejects decreasing timestamps.
    """

    pass
