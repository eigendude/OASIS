################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Ring buffer for time-ordered AHRS measurement nodes.

Responsibility:
    Provide bounded storage for measurement nodes keyed by t_meas, including
    duplicate rejection and diagnostics hooks.

Inputs/outputs:
    - Inputs: measurement packets with t_meas keys.
    - Outputs: ordered nodes suitable for replay to the filter frontier.

Dependencies:
    - Uses Timeline nodes for storage.
    - Depends on TimeBase for timestamp handling.

Determinism:
    - Equality is exact on t_meas; nodes are not merged across timestamps.
    - Duplicate same-type at identical t_meas must be rejected deterministically.
"""


class RingBuffer:
    """Bounded ring buffer of measurement timeline nodes.

    Purpose:
        Store time-keyed measurement nodes and enforce duplicate rejection and
        drop policies prior to replay.

    Public API (to be implemented):
        - insert(node)
        - get(t_meas)
        - drop_before(t_cutoff)
        - size()

    Data contract:
        - Nodes are keyed by exact t_meas values.
        - A node may include <=1 IMU packet and <=1 mag packet.

    Frames and units:
        - Times are in seconds or integer nanoseconds as defined by TimeBase.

    Determinism and edge cases:
        - Duplicate same-type at identical t_meas is rejected and diagnosed.
        - Out-of-order inserts trigger replay in ReplayEngine.

    Equations:
        - No equations; this module enforces buffer rules.

    Numerical stability notes:
        - None; operations are discrete and deterministic.

    Suggested unit tests:
        - Duplicate IMU at same t_meas is rejected.
        - Old nodes are dropped when capacity is exceeded.
    """

    pass
