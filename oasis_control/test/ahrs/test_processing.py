################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for AHRS-specific processing and diagnostics helpers."""

from __future__ import annotations

import math

from oasis_control.localization.ahrs.data.diagnostics import AhrsDiagnosticsState
from oasis_control.localization.ahrs.data.diagnostics import snapshot_diagnostics
from oasis_control.localization.ahrs.processing.attitude_mapper import map_imu_to_base
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    GravityConsistencyPolicy,
)
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    evaluate_gravity_consistency,
)
from oasis_control.localization.ahrs.processing.output_adapter import make_ahrs_output
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.data.imu_sample import ImuSample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import make_mounting_transform
from oasis_control.localization.common.measurements.gravity_direction import (
    compute_gravity_direction_residual,
)


def _make_mounting_transform() -> MountingTransform:
    return make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _make_imu_sample() -> ImuSample:
    return ImuSample(
        timestamp_ns=100,
        frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_rad2=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        orientation_covariance_unknown=False,
        angular_velocity_rads=(0.1, 0.2, 0.3),
        angular_velocity_covariance_rads2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        linear_acceleration_mps2=(0.0, 0.0, -9.81),
        linear_acceleration_covariance_mps2_2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )


def test_snapshot_diagnostics_uses_nan_without_residual() -> None:
    state: AhrsDiagnosticsState = AhrsDiagnosticsState(
        dropped_stale_imu_count=1,
        dropped_stale_gravity_count=2,
        gravity_rejection_count=3,
        gravity_gated_in=False,
        gravity_rejected=True,
        transform_lookup_failure_count=4,
        invalid_mounting_transform_count=5,
        last_mounting_lookup_error="lookup_error",
        last_gravity_rejection_reason="residual_norm",
    )

    snapshot = snapshot_diagnostics(state)

    assert math.isnan(snapshot.gravity_residual_norm)
    assert math.isnan(snapshot.gravity_mahalanobis_distance)
    assert snapshot.dropped_stale_imu_count == 1
    assert snapshot.dropped_stale_gravity_count == 2
    assert snapshot.gravity_rejection_count == 3
    assert snapshot.gravity_gated_in is False
    assert snapshot.gravity_rejected is True
    assert snapshot.transform_lookup_failure_count == 4
    assert snapshot.invalid_mounting_transform_count == 5
    assert snapshot.last_mounting_lookup_error == "lookup_error"
    assert snapshot.last_gravity_rejection_reason == "residual_norm"


def test_snapshot_diagnostics_propagates_residual_values() -> None:
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(0.0, 0.0, -9.81),
        measured_gravity_covariance_mps2_2=(
            (0.04, 0.0, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, 0.0, 0.04),
        ),
        mounted_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    state: AhrsDiagnosticsState = AhrsDiagnosticsState(last_gravity_residual=residual)

    snapshot = snapshot_diagnostics(state)

    assert snapshot.gravity_residual_norm == residual.residual_norm
    assert snapshot.gravity_mahalanobis_distance == residual.mahalanobis_distance


def test_make_ahrs_output_bundles_no_residual_case() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())

    output = make_ahrs_output(mounted_sample, None)

    assert output.mounted_imu == mounted_sample
    assert output.gravity_residual is None


def test_make_ahrs_output_bundles_with_residual_case() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())
    residual = compute_gravity_direction_residual(
        measured_gravity_mps2=(0.0, 0.0, -9.81),
        measured_gravity_covariance_mps2_2=(
            (0.04, 0.0, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, 0.0, 0.04),
        ),
        mounted_orientation_xyzw=mounted_sample.orientation_xyzw,
    )

    output = make_ahrs_output(mounted_sample, residual)

    assert output.mounted_imu == mounted_sample
    assert output.gravity_residual == residual


def test_map_imu_to_base_wraps_mounting_behavior() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())

    assert mounted_sample.frame_id == "base_link"
    assert mounted_sample.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)


def test_gravity_consistency_accepts_low_residual() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())
    policy = GravityConsistencyPolicy(
        residual_norm_threshold=0.35,
        mahalanobis_distance_threshold=5.0,
    )

    decision = evaluate_gravity_consistency(
        gravity_sample=GravitySample(
            timestamp_ns=101,
            frame_id="imu_link",
            gravity_mps2=(0.01, 0.0, -9.81),
            gravity_covariance_mps2_2=(
                (0.04, 0.0, 0.0),
                (0.0, 0.04, 0.0),
                (0.0, 0.0, 0.04),
            ),
        ),
        mounted_imu_sample=mounted_sample,
        mounting_transform=_make_mounting_transform(),
        policy=policy,
    )

    assert decision.accepted is True
    assert decision.rejection_reason == ""


def test_gravity_consistency_accepts_small_residual_with_overconfident_covariance() -> (
    None
):
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())
    policy = GravityConsistencyPolicy(
        residual_norm_threshold=0.35,
        mahalanobis_distance_threshold=1.0,
    )

    decision = evaluate_gravity_consistency(
        gravity_sample=GravitySample(
            timestamp_ns=101,
            frame_id="imu_link",
            gravity_mps2=(0.05, 0.0, -9.81),
            gravity_covariance_mps2_2=(
                (1.0e-6, 0.0, 0.0),
                (0.0, 1.0e-6, 0.0),
                (0.0, 0.0, 1.0e-6),
            ),
        ),
        mounted_imu_sample=mounted_sample,
        mounting_transform=_make_mounting_transform(),
        policy=policy,
    )

    assert decision.accepted is True
    assert decision.rejection_reason == ""
    assert math.isfinite(decision.residual.mahalanobis_distance)
    assert decision.residual.mahalanobis_distance > 1.0


def test_gravity_consistency_rejects_by_residual_norm() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())
    policy = GravityConsistencyPolicy(
        residual_norm_threshold=0.35,
        mahalanobis_distance_threshold=50.0,
    )

    decision = evaluate_gravity_consistency(
        gravity_sample=GravitySample(
            timestamp_ns=102,
            frame_id="imu_link",
            gravity_mps2=(0.0, 9.81, 0.0),
            gravity_covariance_mps2_2=(
                (0.04, 0.0, 0.0),
                (0.0, 0.04, 0.0),
                (0.0, 0.0, 0.04),
            ),
        ),
        mounted_imu_sample=mounted_sample,
        mounting_transform=_make_mounting_transform(),
        policy=policy,
    )

    assert decision.accepted is False
    assert decision.rejection_reason == "residual_norm"


def test_gravity_consistency_keeps_mahalanobis_as_diagnostic_only() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())
    policy = GravityConsistencyPolicy(
        residual_norm_threshold=0.35,
        mahalanobis_distance_threshold=1.0,
    )

    decision = evaluate_gravity_consistency(
        gravity_sample=GravitySample(
            timestamp_ns=103,
            frame_id="imu_link",
            gravity_mps2=(0.05, 0.0, -9.81),
            gravity_covariance_mps2_2=(
                (1.0e-6, 0.0, 0.0),
                (0.0, 1.0e-6, 0.0),
                (0.0, 0.0, 1.0e-6),
            ),
        ),
        mounted_imu_sample=mounted_sample,
        mounting_transform=_make_mounting_transform(),
        policy=policy,
    )

    assert decision.accepted is True
    assert decision.rejection_reason == ""
    assert math.isfinite(decision.residual.mahalanobis_distance)
    assert (
        decision.residual.mahalanobis_distance > policy.mahalanobis_distance_threshold
    )


def test_gravity_consistency_accepts_without_covariance_when_norm_is_small() -> None:
    mounted_sample = map_imu_to_base(_make_imu_sample(), _make_mounting_transform())
    policy = GravityConsistencyPolicy(
        residual_norm_threshold=0.35,
        mahalanobis_distance_threshold=1.0,
    )

    decision = evaluate_gravity_consistency(
        gravity_sample=GravitySample(
            timestamp_ns=104,
            frame_id="imu_link",
            gravity_mps2=(0.01, 0.0, -9.81),
            gravity_covariance_mps2_2=None,
        ),
        mounted_imu_sample=mounted_sample,
        mounting_transform=_make_mounting_transform(),
        policy=policy,
    )

    assert decision.accepted is True
    assert math.isnan(decision.residual.mahalanobis_distance)
    assert decision.rejection_reason == ""
