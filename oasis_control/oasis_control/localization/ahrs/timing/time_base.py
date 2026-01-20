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
    - Outputs: normalized timestamps used by buffers and replay.

Dependencies:
    - Used by RingBuffer, Timeline, and ReplayEngine.

Determinism:
    Time conversions must be deterministic; no implicit wall clock access.
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
        - t_meas: measurement timestamp associated with a sensor packet.
        - t_now: current wall-clock timestamp (provided externally).
        - t_filter: filter frontier timestamp after processing.

    Frames and units:
        - All time values are seconds as float or integer nanoseconds.

    Determinism and edge cases:
        - Timestamp equality is exact; no epsilon-based merging.
        - Validation should reject negative or NaN timestamps.

    Equations:
        - No equations; this module defines time terminology.

    Numerical stability notes:
        - Avoid float rounding when converting between units.

    Suggested unit tests:
        - to_seconds/from_seconds round-trip consistency.
        - validate_monotonic rejects decreasing timestamps.
    """

    pass
