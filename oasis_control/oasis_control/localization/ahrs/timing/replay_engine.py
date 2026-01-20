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
    - Inputs: timeline nodes keyed by t_meas.
    - Outputs: updated filter state and diagnostics counts.

Dependencies:
    - Uses RingBuffer and Timeline for storage.
    - Interfaces with AhrsEkf for updates.

Determinism:
    - Out-of-order insert triggers replay forward to the frontier.
    - Publish only when the frontier advances.
"""


class ReplayEngine:
    """Replay logic for time-ordered AHRS measurements.

    Purpose:
        Deterministically replay buffered measurements when data arrives out
        of order and manage the filter frontier time.

    Public API (to be implemented):
        - insert_imu(imu_packet)
        - insert_mag(mag_packet)
        - replay_from(t_meas)
        - frontier_time()

    Data contract:
        - Nodes are keyed by t_meas and contain <=1 measurement per type.
        - Duplicate same-type at same t_meas must be rejected and diagnosed.

    Frames and units:
        - All timestamps follow TimeBase conventions.

    Determinism and edge cases:
        - Out-of-order insert => replay forward to frontier.
        - Out-of-order insert does not trigger immediate publish.
        - Publish only when frontier advances.

    Equations:
        - No equations; procedural replay rules are specified above.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - Out-of-order insert triggers replay.
        - Duplicate measurements are rejected with diagnostics.
    """

    pass
