################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
from typing import Optional

from oasis_control.localization.ahrs.ahrs_types.imu_packet import ImuPacket
from oasis_control.localization.ahrs.ahrs_types.mag_packet import MagPacket
from oasis_control.localization.ahrs.ahrs_types.stationary_packet import (
    StationaryPacket,
)
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.timing.time_base import TimeBase


@dataclass(slots=True)
class TimelineNode:
    """Time-keyed node containing measurements for a single timestamp.

    Purpose:
        Aggregate IMU and magnetometer measurements that share the same
        t_meas_ns into a single node for replay, while holding the posterior
        mean state and covariance after applying all updates at t_meas_ns.

    Responsibility:
        Define a time-keyed node that can hold multiple measurement types at a
        single t_meas_ns without merging across timestamps, plus the
        posterior state and covariance stored at that time.

    Inputs/outputs:
        - Inputs: ImuPacket or MagPacket instances keyed by t_meas_ns.
        - Outputs: node with AhrsState/AhrsCovariance and optional measurement
          slots.

    Dependencies:
        - Used by RingBuffer and ReplayEngine.

    Public API:
        - insert_imu(imu_packet)
        - insert_mag(mag_packet)
        - insert_stationary(stationary_packet)
        - is_complete()
        - time_ns()

    Data contract:
        - t_meas_ns: timestamp key in integer nanoseconds.
        - state: AhrsState posterior mean after updates at t_meas_ns.
        - covariance: AhrsCovariance posterior after updates at t_meas_ns.
        - calibration_prior_applied: whether calibration prior was applied
          by the EKF when computing this node's posterior.
        - imu_packet: Optional[ImuPacket].
        - mag_packet: Optional[MagPacket].
        - stationary_packet: Optional[StationaryPacket].
        - Each node holds at most one IMU packet and one mag packet.
        - Each node holds at most one stationary packet.
        - IMU and mag can share a node when t_meas_ns matches exactly.
        - Duplicate same-type measurements at the same t_meas_ns are rejected
          deterministically (no replacement/merge).

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
        - insert_stationary rejects second stationary packet at the same
          t_meas_ns.
    """

    t_meas_ns: int
    state: Optional[AhrsState] = None
    covariance: Optional[AhrsCovariance] = None
    calibration_prior_applied: Optional[bool] = None
    imu_packet: Optional[ImuPacket] = None
    mag_packet: Optional[MagPacket] = None
    stationary_packet: Optional[StationaryPacket] = None
    diagnostics: dict[str, int] = field(
        default_factory=lambda: {
            "duplicate_imu": 0,
            "duplicate_mag": 0,
            "duplicate_stationary": 0,
        }
    )

    def __post_init__(self) -> None:
        """Validate the timestamp key for this node."""
        TimeBase.validate_non_negative(self.t_meas_ns)

    def insert_imu(self, packet: ImuPacket) -> bool:
        """Insert an IMU packet for this timestamp."""
        self._validate_packet_time_match(packet.t_meas_ns)
        if self.imu_packet is not None:
            self.diagnostics["duplicate_imu"] += 1
            return False
        self.imu_packet = packet
        return True

    def insert_mag(self, packet: MagPacket) -> bool:
        """Insert a magnetometer packet for this timestamp."""
        self._validate_packet_time_match(packet.t_meas_ns)
        if self.mag_packet is not None:
            self.diagnostics["duplicate_mag"] += 1
            return False
        self.mag_packet = packet
        return True

    def insert_stationary(self, packet: StationaryPacket) -> bool:
        """Insert a stationary packet for this timestamp."""
        self._validate_packet_time_match(packet.t_meas_ns)
        if self.stationary_packet is not None:
            self.diagnostics["duplicate_stationary"] += 1
            return False
        self.stationary_packet = packet
        return True

    def is_complete(self) -> bool:
        """Return True when state and covariance are present."""
        return self.state is not None and self.covariance is not None

    def time_ns(self) -> int:
        """Return the timestamp key in nanoseconds."""
        return self.t_meas_ns

    def _validate_packet_time_match(self, t_meas_ns: int) -> None:
        TimeBase.validate_non_negative(t_meas_ns)
        if t_meas_ns != self.t_meas_ns:
            raise ValueError("t_meas_ns mismatch")
