################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class StationaryPacket:
    """Stationary detection result packet for deterministic pseudo-updates.

    Responsibility:
        Document the stationary detection output used to drive ZUPT and
        optional no-turn pseudo-measurements in the AHRS core.

    Purpose:
        Provide a deterministic, time-keyed container for stationary
        detection decisions and associated measurement covariances.

    Inputs/outputs:
        - Inputs: windowed IMU statistics and stationary decision metadata.
        - Outputs: StationaryPacket with full 3x3 covariances for ZUPT and
          optional no-turn updates.

    Dependencies:
        - Produced by StationaryDetector.
        - Consumed by ReplayEngine, TimelineNode, and AhrsEkf.

    Public API (to be implemented):
        - validate()
        - as_dict()

    Data contract:
        Required fields:
        - t_meas_ns: representative timestamp in int nanoseconds.
        - window_start_ns: inclusive window start in int nanoseconds.
        - window_end_ns: inclusive window end in int nanoseconds.
        - is_stationary: boolean decision for the window.
        - R_v: ZUPT covariance (3, 3), full SPD.
        - R_omega: optional no-turn covariance (3, 3), full SPD.
        - metadata: dict of deterministic scores/counters/thresholds.

    Frames and units:
        - t_meas_ns and window bounds are int nanoseconds.
        - R_v in (m/s)^2 for velocity in {W}.
        - R_omega in (rad/s)^2 for angular velocity in {B}.
        - Covariances are full matrices; never diagonalize.

    Determinism and edge cases:
        - All timestamps are int nanoseconds since an arbitrary epoch.
        - Exact equality is used for buffer attachment (no epsilon).
        - Identical buffered IMU packets must yield identical packets.
        - validate() must reject non-SPD R_v/R_omega or wrong shapes.
        - metadata must avoid nondeterministic fields (e.g., receipt time).

    Equations:
        - None; this is a data container.

    Numerical stability notes:
        - Covariances must be clamped with LinearAlgebra.clamp_spd.

    Suggested unit tests:
        - validate rejects non-SPD covariances.
        - Equality of t_meas_ns keys triggers exact attachment.
    """

    pass
