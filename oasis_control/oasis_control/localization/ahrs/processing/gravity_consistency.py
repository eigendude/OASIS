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

from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.frames.mounting import MountedGravitySample
from oasis_control.localization.common.frames.mounting import MountedImuSample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import apply_mounting_to_gravity
from oasis_control.localization.common.measurements.gravity_direction import (
    GravityDirectionResidual,
)
from oasis_control.localization.common.measurements.gravity_direction import (
    compute_gravity_direction_residual,
)


@dataclass(frozen=True)
class GravityConsistencyPolicy:
    """
    AHRS gravity consistency thresholds.

    The AHRS always publishes the mounted IMU output, but it only marks the
    latest gravity sample as gated in when the absolute gravity-direction
    residual remains below the configured threshold.

    Mahalanobis distance stays available in diagnostics, but AHRS gating does
    not reject on it because normalized gravity-direction covariances can be
    unrealistically tight for this consistency check.
    """

    residual_norm_threshold: float
    mahalanobis_distance_threshold: float


@dataclass(frozen=True)
class GravityConsistencyDecision:
    """
    Result of evaluating one gravity consistency check against AHRS policy.

    Fields:
        residual: raw gravity consistency residual summary
        accepted: true when the latest gravity sample passed the AHRS gate
        rejection_reason: short machine-facing reason when accepted is false
    """

    residual: GravityDirectionResidual
    accepted: bool
    rejection_reason: str


def evaluate_gravity_consistency(
    *,
    gravity_sample: GravitySample,
    mounted_imu_sample: MountedImuSample,
    mounting_transform: MountingTransform,
    policy: GravityConsistencyPolicy,
) -> GravityConsistencyDecision:
    """
    Compare the latest valid gravity sample against the mounted attitude output.
    """

    mounted_gravity_sample: MountedGravitySample = apply_mounting_to_gravity(
        gravity_sample,
        mounting_transform,
    )
    residual: GravityDirectionResidual = compute_gravity_direction_residual(
        measured_gravity_mps2=mounted_gravity_sample.gravity_mps2,
        measured_gravity_covariance_mps2_2=(
            mounted_gravity_sample.gravity_covariance_mps2_2
        ),
        mounted_orientation_xyzw=mounted_imu_sample.orientation_xyzw,
    )
    if residual.residual_norm > policy.residual_norm_threshold:
        return GravityConsistencyDecision(
            residual=residual,
            accepted=False,
            rejection_reason="residual_norm",
        )

    # AHRS uses the absolute unit-direction mismatch as the primary gate.
    # This stays robust at rest even when the reported gravity covariance is
    # overconfident after normalization into direction space.
    #
    # Keep Mahalanobis published for diagnostics, but do not let it veto an
    # otherwise good stationary gravity sample in this runtime contract.
    _ = policy.mahalanobis_distance_threshold

    return GravityConsistencyDecision(
        residual=residual,
        accepted=True,
        rejection_reason="",
    )
