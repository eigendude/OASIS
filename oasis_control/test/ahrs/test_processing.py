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
from typing import Iterable

from oasis_control.localization.ahrs.data.diagnostics import AhrsDiagnosticsState
from oasis_control.localization.ahrs.data.diagnostics import snapshot_diagnostics
from oasis_control.localization.ahrs.processing.attitude_mapper import map_imu_to_base
from oasis_control.localization.ahrs.processing.boot_mounting_calibrator import (
    BootMountingCalibrator,
)
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    GravityConsistencyPolicy,
)
from oasis_control.localization.ahrs.processing.gravity_consistency import (
    evaluate_gravity_consistency,
)
from oasis_control.localization.ahrs.processing.output_adapter import make_ahrs_output
from oasis_control.localization.common.algebra.quat import rotate_vector
from oasis_control.localization.common.algebra.quat import transpose_matrix
from oasis_control.localization.common.data.gravity_sample import GravitySample
from oasis_control.localization.common.data.imu_sample import ImuSample
from oasis_control.localization.common.frames.mounting import MountingTransform
from oasis_control.localization.common.frames.mounting import apply_mounting_to_gravity
from oasis_control.localization.common.frames.mounting import make_mounting_transform
from oasis_control.localization.common.measurements.gravity_direction import (
    compute_gravity_direction_residual,
)


FULL_ORIENTATION_COVARIANCE: tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
] = (
    (4.0, 1.0, 0.5),
    (1.0, 3.0, -0.25),
    (0.5, -0.25, 2.0),
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
        orientation_covariance_rad2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
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


def _make_full_covariance_imu_sample() -> ImuSample:
    return ImuSample(
        timestamp_ns=100,
        frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_rad2=FULL_ORIENTATION_COVARIANCE,
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


def _quarter_turn_about_z_mounting_transform() -> MountingTransform:
    return make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=(
            0.0,
            0.0,
            math.sin(math.pi / 4.0),
            math.cos(math.pi / 4.0),
        ),
    )


def _quaternion_inverse(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return (
        -quaternion_xyzw[0],
        -quaternion_xyzw[1],
        -quaternion_xyzw[2],
        quaternion_xyzw[3],
    )


def _gravity_in_imu_for_level_base(
    mounting_transform: MountingTransform,
) -> tuple[float, float, float]:
    return rotate_vector(
        transpose_matrix(mounting_transform.rotation_matrix),
        (0.0, 0.0, -9.81),
    )


def _assert_matrix_close(
    lhs_matrix: Iterable[Iterable[float]],
    rhs_matrix: Iterable[Iterable[float]],
    *,
    abs_tol: float = 1.0e-9,
) -> None:
    lhs_rows: tuple[tuple[float, ...], ...] = tuple(
        tuple(float(value) for value in row) for row in lhs_matrix
    )
    rhs_rows: tuple[tuple[float, ...], ...] = tuple(
        tuple(float(value) for value in row) for row in rhs_matrix
    )
    assert len(lhs_rows) == len(rhs_rows)

    for lhs_row, rhs_row in zip(lhs_rows, rhs_rows):
        assert len(lhs_row) == len(rhs_row)
        for lhs_value, rhs_value in zip(lhs_row, rhs_row):
            assert math.isclose(lhs_value, rhs_value, abs_tol=abs_tol)


def _trace(matrix: tuple[tuple[float, ...], ...]) -> float:
    return float(matrix[0][0] + matrix[1][1] + matrix[2][2])


def _determinant(matrix: tuple[tuple[float, ...], ...]) -> float:
    return float(
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _sum_of_principal_minors(matrix: tuple[tuple[float, ...], ...]) -> float:
    trace_value: float = _trace(matrix)
    trace_of_square: float = sum(
        matrix[row_index][column_index] * matrix[column_index][row_index]
        for row_index in range(3)
        for column_index in range(3)
    )
    return 0.5 * (trace_value * trace_value - trace_of_square)


def test_snapshot_diagnostics_uses_nan_without_residual() -> None:
    state: AhrsDiagnosticsState = AhrsDiagnosticsState(
        dropped_stale_imu_count=1,
        dropped_stale_gravity_count=2,
        gravity_rejection_count=3,
        gravity_gated_in=False,
        gravity_rejected=True,
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


def test_boot_mounting_and_runtime_mapping_keep_level_base_level() -> None:
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=(
            0.25488700224417876,
            0.16773125949652065,
            -0.044943455527547777,
            0.9512512425641978,
        ),
    )
    gravity_imu = _gravity_in_imu_for_level_base(mounting_transform)

    calibrator = BootMountingCalibrator(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        calibration_duration_sec=0.0,
        min_sample_count=1,
    )
    solution = calibrator.add_gravity_sample(
        GravitySample(
            timestamp_ns=1,
            frame_id="imu_link",
            gravity_mps2=gravity_imu,
            gravity_covariance_mps2_2=None,
        )
    )

    assert solution is not None

    level_boot_imu_sample = ImuSample(
        timestamp_ns=2,
        frame_id="imu_link",
        orientation_xyzw=_quaternion_inverse(
            solution.mounting_transform.quaternion_xyzw
        ),
        orientation_covariance_rad2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        orientation_covariance_unknown=False,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_rads2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        linear_acceleration_mps2=gravity_imu,
        linear_acceleration_covariance_mps2_2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )

    mounted_imu_sample = map_imu_to_base(
        level_boot_imu_sample,
        solution.mounting_transform,
    )
    mounted_gravity_sample = apply_mounting_to_gravity(
        GravitySample(
            timestamp_ns=1,
            frame_id="imu_link",
            gravity_mps2=gravity_imu,
            gravity_covariance_mps2_2=None,
        ),
        solution.mounting_transform,
    )
    assert math.isclose(mounted_gravity_sample.gravity_mps2[0], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_gravity_sample.gravity_mps2[1], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_gravity_sample.gravity_mps2[2], -9.81, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[0], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[1], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[2], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[3], 1.0, abs_tol=1.0e-6)


def test_map_imu_to_base_does_not_relabel_unmounted_orientation() -> None:
    mounting_transform = make_mounting_transform(
        parent_frame_id="base_link",
        child_frame_id="imu_link",
        quaternion_xyzw=(
            0.25488700224417876,
            0.16773125949652065,
            -0.044943455527547777,
            0.9512512425641978,
        ),
    )
    imu_orientation_xyzw = _quaternion_inverse(mounting_transform.quaternion_xyzw)
    imu_sample = ImuSample(
        timestamp_ns=3,
        frame_id="imu_link",
        orientation_xyzw=imu_orientation_xyzw,
        orientation_covariance_rad2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        orientation_covariance_unknown=False,
        angular_velocity_rads=(0.0, 0.0, 0.0),
        angular_velocity_covariance_rads2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        linear_acceleration_mps2=_gravity_in_imu_for_level_base(mounting_transform),
        linear_acceleration_covariance_mps2_2=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
    )

    mounted_imu_sample = map_imu_to_base(
        imu_sample,
        mounting_transform,
    )

    assert abs(imu_sample.orientation_xyzw[0]) > 0.1
    assert abs(imu_sample.orientation_xyzw[1]) > 0.1
    assert math.isclose(mounted_imu_sample.orientation_xyzw[0], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[1], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[2], 0.0, abs_tol=1.0e-6)
    assert math.isclose(mounted_imu_sample.orientation_xyzw[3], 1.0, abs_tol=1.0e-6)


def test_map_imu_to_base_keeps_identity_orientation_covariance_unchanged() -> None:
    mounted_sample = map_imu_to_base(
        _make_full_covariance_imu_sample(),
        _make_mounting_transform(),
    )

    assert mounted_sample.orientation_covariance_rad2 is not None
    _assert_matrix_close(
        mounted_sample.orientation_covariance_rad2,
        FULL_ORIENTATION_COVARIANCE,
    )


def test_map_imu_to_base_rotates_full_orientation_covariance() -> None:
    mounted_sample = map_imu_to_base(
        _make_full_covariance_imu_sample(),
        _quarter_turn_about_z_mounting_transform(),
    )

    expected_covariance = (
        (3.0, -1.0, 0.25),
        (-1.0, 4.0, 0.5),
        (0.25, 0.5, 2.0),
    )

    assert mounted_sample.orientation_covariance_rad2 is not None
    _assert_matrix_close(
        mounted_sample.orientation_covariance_rad2,
        expected_covariance,
    )


def test_map_imu_to_base_preserves_full_covariance_invariants() -> None:
    mounted_sample = map_imu_to_base(
        _make_full_covariance_imu_sample(),
        _quarter_turn_about_z_mounting_transform(),
    )

    assert mounted_sample.orientation_covariance_rad2 is not None
    rotated_covariance = mounted_sample.orientation_covariance_rad2

    _assert_matrix_close(
        rotated_covariance,
        (
            (
                rotated_covariance[0][0],
                rotated_covariance[1][0],
                rotated_covariance[2][0],
            ),
            (
                rotated_covariance[0][1],
                rotated_covariance[1][1],
                rotated_covariance[2][1],
            ),
            (
                rotated_covariance[0][2],
                rotated_covariance[1][2],
                rotated_covariance[2][2],
            ),
        ),
    )
    assert math.isclose(
        _trace(rotated_covariance),
        _trace(FULL_ORIENTATION_COVARIANCE),
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        _sum_of_principal_minors(rotated_covariance),
        _sum_of_principal_minors(FULL_ORIENTATION_COVARIANCE),
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        _determinant(rotated_covariance),
        _determinant(FULL_ORIENTATION_COVARIANCE),
        abs_tol=1.0e-9,
    )


def test_map_imu_to_base_preserves_unknown_orientation_covariance() -> None:
    unknown_covariance_sample = ImuSample(
        timestamp_ns=100,
        frame_id="imu_link",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        orientation_covariance_rad2=None,
        orientation_covariance_unknown=True,
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

    mounted_sample = map_imu_to_base(
        unknown_covariance_sample,
        _quarter_turn_about_z_mounting_transform(),
    )

    assert mounted_sample.orientation_covariance_unknown is True
    assert mounted_sample.orientation_covariance_rad2 is None


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
