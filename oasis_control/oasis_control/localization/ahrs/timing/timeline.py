################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Timeline node definition for AHRS measurements.

Responsibility:
    Define a time-keyed node that can hold multiple measurement types at a
    single t_meas without merging across timestamps.

Inputs/outputs:
    - Inputs: ImuPacket or MagPacket instances.
    - Outputs: node with optional imu_packet and mag_packet slots.

Dependencies:
    - Used by RingBuffer and ReplayEngine.

Determinism:
    - Nodes are keyed by exact t_meas equality.
    - At most one measurement per type slot is allowed.
"""


class Timeline:
    """Time-keyed node containing measurements for a single timestamp.

    Purpose:
        Aggregate IMU and magnetometer measurements that share the same
        t_meas into a single node for replay.

    Public API (to be implemented):
        - insert_imu(imu_packet)
        - insert_mag(mag_packet)
        - is_complete()
        - t_meas()

    Data contract:
        - Each node holds at most one IMU packet and one mag packet.
        - Duplicate same-type measurements at the same t_meas are rejected.

    Frames and units:
        - t_meas is in the units defined by TimeBase.

    Determinism and edge cases:
        - No merging of different t_meas values.
        - Duplicate insert attempts must trigger diagnostics hooks.

    Equations:
        - None; this module defines storage behavior.

    Numerical stability notes:
        - None.

    Suggested unit tests:
        - insert_imu rejects second IMU at the same t_meas.
        - insert_mag rejects second mag at the same t_meas.
    """

    pass
