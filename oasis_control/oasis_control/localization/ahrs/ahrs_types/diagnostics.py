################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Diagnostics payload definitions for the AHRS core.

Responsibility:
    Document the diagnostics fields produced by buffering and replay logic.

Inputs/outputs:
    - Inputs: internal counters and timing state.
    - Outputs: structured diagnostics for integration layers.

Dependencies:
    - Produced by RingBuffer and ReplayEngine.

Determinism:
    Diagnostics counters must increment deterministically for identical input
    sequences.
"""


class AhrsDiagnostics:
    """Diagnostics data for AHRS buffering and replay.

    Purpose:
        Provide a structured summary of buffer and replay behavior for logging
        and monitoring.

    Public API (to be implemented):
        - as_dict()
        - reset_counters()

    Data contract:
        Required fields:
        - buffer_size: current number of nodes in the buffer.
        - t_filter: current filter frontier timestamp.
        - drop_count: number of dropped measurements.
        - duplicate_count: number of rejected duplicates.
        - replay_count: number of replay operations.
        - last_replay_from: timestamp of the last replay start.

    Frames and units:
        - Time fields use TimeBase units.

    Determinism and edge cases:
        - Counters are monotonic and updated deterministically.

    Equations:
        - None; this is a diagnostics container.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - duplicate_count increments on duplicate insertion.
        - replay_count increments when replay occurs.
    """

    pass
