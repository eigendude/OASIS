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
    - Inputs: raw timestamps or floating-point seconds for conversion.
    - Outputs: canonical integer nanoseconds (t_ns) used by buffers/replay.

Dependencies:
    - Used by RingBuffer, Timeline, and ReplayEngine.

Determinism:
    Canonical internal representation is int nanoseconds (t_ns).
    Equality for keying uses exact integer equality with no epsilon.
    The epoch is irrelevant because only differences and equality are used.
"""


class TimeBase:
    """Time base helpers for AHRS buffering and replay logic.

    Purpose:
        Provide a consistent definition of measurement timestamps and the
        filter frontier time used by buffering and replay.

    Public API (to be implemented):
        - to_seconds(t_ns)
        - from_seconds(t_sec)
        - validate_monotonic(t_prev_ns, t_next_ns)

    Data contract:
        - t_meas_ns: measurement timestamp in integer nanoseconds.
        - t_now_ns: current wall-clock timestamp (provided externally).
        - t_filter_ns: filter frontier timestamp after processing.

    Frames and units:
        - Canonical storage is int nanoseconds (t_ns).
        - Boundary helpers may convert to/from float seconds for
          convenience. Float seconds must never be used for keying or
          storage.

    Determinism and edge cases:
        - Timestamp equality is exact integer equality for keying.
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - Validation should reject negative timestamps and NaN seconds
          inputs.
        - from_seconds(t_sec) converts float seconds to int nanoseconds by
          rounding to the nearest integer with ties to even.
        - to_seconds(t_ns) converts int nanoseconds to float seconds and is
          lossy.
        - Float conversions must be explicit and never used for key
          comparisons.

    Equations:
        - No equations; this module defines time terminology.

    Numerical stability notes:
        - Avoid float rounding when converting between units.

    Suggested unit tests:
        - to_seconds/from_seconds round-trip consistency.
        - validate_monotonic rejects decreasing timestamps.
    """

    pass
