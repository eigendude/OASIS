################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Timeline node definition for AHRS measurements and state snapshots.

Responsibility:
    Define a time-keyed node that can hold multiple measurement types at a
    single t_meas_ns without merging across timestamps, plus the mean state
    and covariance stored at that time.

Inputs/outputs:
    - Inputs: ImuPacket or MagPacket instances keyed by t_meas_ns.
    - Outputs: node with AhrsState/AhrsCovariance and optional measurement
      slots.

Dependencies:
    - Used by RingBuffer and ReplayEngine.

Determinism:
    - Nodes are keyed by exact t_meas_ns integer equality.
    - At most one measurement per type slot is allowed.
    - No epsilon merging, rounding, or "close enough" matching is allowed.
"""


class Timeline:
    """Time-keyed node containing measurements for a single timestamp.

    Purpose:
        Aggregate IMU and magnetometer measurements that share the same
        t_meas_ns into a single node for replay, while holding the mean state
        and covariance at that timestamp.

    Public API (to be implemented):
        - insert_imu(imu_packet)
        - insert_mag(mag_packet)
        - is_complete()
        - t_meas_ns()

    Data contract:
        - t_meas_ns: timestamp key in integer nanoseconds.
        - state: AhrsState mean at t_meas_ns.
        - covariance: AhrsCovariance at t_meas_ns.
        - imu_packet: Optional[ImuPacket].
        - mag_packet: Optional[MagPacket].
        - Each node holds at most one IMU packet and one mag packet.
        - IMU and mag can share a node when t_meas_ns matches exactly.
        - Duplicate same-type measurements at the same t_meas_ns are rejected
          deterministically.

    Frames and units:
        - t_meas_ns uses TimeBase canonical int nanoseconds.

    Determinism and edge cases:
        - All timestamps are int nanoseconds since an arbitrary epoch.
          The epoch is irrelevant because only differences and exact
          equality are used.
        - Exact t_meas_ns match determines node attachment.
        - No merging of different t_meas_ns values.
        - Duplicate insert attempts must be rejected and increment
          diagnostics. Never merge duplicates.

    Equations:
        - None; this module defines storage behavior.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - insert_imu rejects second IMU at the same t_meas_ns.
        - insert_mag rejects second mag at the same t_meas_ns.
    """

    pass
