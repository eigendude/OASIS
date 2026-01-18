################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class MagPacket:
    """Magnetometer measurement packet definition for the AHRS core.

    Responsibility:
        Document the magnetometer measurement fields required by the AHRS
        filter without introducing ROS message types.

    Purpose:
        Provide a data container for magnetometer samples with covariance and
        frame identifiers.

    Inputs/outputs:
        - Inputs: raw magnetometer measurements with covariance.
        - Outputs: structured packet used by MagModel and AhrsEkf.

    Dependencies:
        - Consumed by MagModel and ReplayEngine.

    Public API (to be implemented):
        - validate()
        - as_dict()

    Data contract:
        Required fields:
        - t_meas_ns: measurement timestamp in int nanoseconds since an
          arbitrary epoch.
        - frame_id: sensor frame identifier for {M}.
        - z_m: magnetometer measurement vector (3,).
        - R_m_raw: raw measurement covariance (3, 3).

    Frames and units:
        - z_m in tesla, frame {M}.
        - R_m_raw in tesla^2.

    Determinism and edge cases:
        - Packets are immutable once created; fields must not change during
          replay.
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - t_meas_ns equality is exact for buffer keying.
        - No epsilon merging or rounding-based matching is permitted.
        - Missing covariance must be rejected.

    Equations:
        - No equations; this is a data container.

    Numerical stability notes:
        - Ensure R_m_raw is SPD.

    Suggested unit tests:
        - validate rejects wrong shapes.
        - validate rejects non-SPD covariance.
    """

    pass
