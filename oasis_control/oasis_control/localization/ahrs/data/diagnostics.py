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

import math
from dataclasses import dataclass
from typing import Optional

from oasis_control.localization.common.measurements.gravity_direction import (
    GravityDirectionResidual,
)


@dataclass
class AhrsDiagnosticsState:
    """
    Mutable AHRS runtime diagnostics shared between node updates and tests.

    Fields:
        accepted_imu_count: count of accepted IMU samples
        accepted_gravity_count: count of accepted gravity samples
        rejected_imu_count: count of rejected IMU samples
        rejected_gravity_count: count of rejected gravity samples
        dropped_stale_imu_count: count of stale IMU samples dropped by policy
        dropped_stale_gravity_count: count of stale gravity samples dropped by
            policy
        gravity_rejection_count: count of gravity consistency checks rejected by
            policy
        has_gravity: true once a valid gravity sample has been cached
        has_mounting: true once T_BI has been solved and cached
        gravity_gated_in: true when the latest gravity consistency check passed
        gravity_rejected: true when the latest gravity consistency check failed
        last_bad_imu_frame_id: most recent IMU frame mismatch
        last_bad_gravity_frame_id: most recent gravity frame mismatch
        last_accepted_imu_timestamp_ns: newest accepted IMU timestamp
        last_accepted_gravity_timestamp_ns: newest accepted gravity timestamp
        last_mounting_lookup_error: short text from the latest mounting
            availability state
        last_gravity_rejection_reason: short text from the latest gravity
            consistency rejection
        last_gravity_residual: most recent gravity consistency summary
    """

    accepted_imu_count: int = 0
    accepted_gravity_count: int = 0
    rejected_imu_count: int = 0
    rejected_gravity_count: int = 0
    dropped_stale_imu_count: int = 0
    dropped_stale_gravity_count: int = 0
    gravity_rejection_count: int = 0
    has_gravity: bool = False
    has_mounting: bool = False
    gravity_gated_in: bool = False
    gravity_rejected: bool = False
    last_bad_imu_frame_id: str = ""
    last_bad_gravity_frame_id: str = ""
    last_accepted_imu_timestamp_ns: Optional[int] = None
    last_accepted_gravity_timestamp_ns: Optional[int] = None
    last_mounting_lookup_error: str = ""
    last_gravity_rejection_reason: str = ""
    last_gravity_residual: Optional[GravityDirectionResidual] = None


@dataclass(frozen=True)
class AhrsDiagnosticsSnapshot:
    """
    Immutable summary used to populate the ROS AHRS status message.

    Fields:
        accepted_imu_count: count of accepted IMU samples
        accepted_gravity_count: count of accepted gravity samples
        rejected_imu_count: count of rejected IMU samples
        rejected_gravity_count: count of rejected gravity samples
        dropped_stale_imu_count: count of stale IMU samples dropped by policy
        dropped_stale_gravity_count: count of stale gravity samples dropped by
            policy
        gravity_rejection_count: count of gravity consistency rejections
        has_gravity: true when a valid gravity sample is cached
        has_mounting: true when T_BI has been solved and cached
        gravity_gated_in: true when the latest gravity consistency check passed
        gravity_rejected: true when the latest gravity consistency check failed
        gravity_residual_norm: latest gravity residual norm or NaN
        gravity_mahalanobis_distance: latest gravity Mahalanobis distance or NaN
        last_mounting_lookup_error: short text from the latest mounting
            availability state
        last_gravity_rejection_reason: short text from the latest gravity
            consistency rejection
    """

    accepted_imu_count: int
    accepted_gravity_count: int
    rejected_imu_count: int
    rejected_gravity_count: int
    dropped_stale_imu_count: int
    dropped_stale_gravity_count: int
    gravity_rejection_count: int
    has_gravity: bool
    has_mounting: bool
    gravity_gated_in: bool
    gravity_rejected: bool
    gravity_residual_norm: float
    gravity_mahalanobis_distance: float
    last_mounting_lookup_error: str
    last_gravity_rejection_reason: str


def snapshot_diagnostics(state: AhrsDiagnosticsState) -> AhrsDiagnosticsSnapshot:
    """
    Freeze the mutable AHRS diagnostics into a message-ready snapshot.
    """

    gravity_residual_norm: float = math.nan
    gravity_mahalanobis_distance: float = math.nan
    if state.last_gravity_residual is not None:
        gravity_residual_norm = state.last_gravity_residual.residual_norm
        gravity_mahalanobis_distance = state.last_gravity_residual.mahalanobis_distance

    return AhrsDiagnosticsSnapshot(
        accepted_imu_count=state.accepted_imu_count,
        accepted_gravity_count=state.accepted_gravity_count,
        rejected_imu_count=state.rejected_imu_count,
        rejected_gravity_count=state.rejected_gravity_count,
        dropped_stale_imu_count=state.dropped_stale_imu_count,
        dropped_stale_gravity_count=state.dropped_stale_gravity_count,
        gravity_rejection_count=state.gravity_rejection_count,
        has_gravity=state.has_gravity,
        has_mounting=state.has_mounting,
        gravity_gated_in=state.gravity_gated_in,
        gravity_rejected=state.gravity_rejected,
        gravity_residual_norm=gravity_residual_norm,
        gravity_mahalanobis_distance=gravity_mahalanobis_distance,
        last_mounting_lookup_error=state.last_mounting_lookup_error,
        last_gravity_rejection_reason=state.last_gravity_rejection_reason,
    )
