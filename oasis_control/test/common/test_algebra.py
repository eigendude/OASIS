################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Tests for reusable algebra helpers used by AHRS/common code."""

from __future__ import annotations

import math

from oasis_control.localization.common.algebra.covariance import (
    embed_linear_covariance_3x3,
)
from oasis_control.localization.common.algebra.covariance import (
    flatten_matrix3_row_major,
)
from oasis_control.localization.common.algebra.covariance import (
    is_positive_semidefinite_diagonal,
)
from oasis_control.localization.common.algebra.covariance import (
    parse_linear_covariance_3x3,
)
from oasis_control.localization.common.algebra.covariance import parse_row_major_matrix3
from oasis_control.localization.common.algebra.covariance import rotate_covariance
from oasis_control.localization.common.algebra.quat import normalize_quaternion_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_conjugate_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_multiply_xyzw
from oasis_control.localization.common.algebra.quat import quaternion_to_rotation_matrix
from oasis_control.localization.common.algebra.quat import rotate_vector
from oasis_control.localization.common.algebra.quat import transpose_matrix


def test_normalize_quaternion_accepts_non_unit_input() -> None:
    normalized_quaternion: tuple[float, float, float, float] | None = (
        normalize_quaternion_xyzw((0.0, 0.0, 0.0, 2.0))
    )

    assert normalized_quaternion == (0.0, 0.0, 0.0, 1.0)


def test_normalize_quaternion_rejects_zero_norm() -> None:
    normalized_quaternion: tuple[float, float, float, float] | None = (
        normalize_quaternion_xyzw((0.0, 0.0, 0.0, 0.0))
    )

    assert normalized_quaternion is None


def test_normalize_quaternion_rejects_non_finite_values() -> None:
    normalized_quaternion: tuple[float, float, float, float] | None = (
        normalize_quaternion_xyzw((0.0, 0.0, 0.0, math.nan))
    )

    assert normalized_quaternion is None


def test_quaternion_multiply_xyzw_composes_rotations() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    composed_quaternion: tuple[float, float, float, float] = quaternion_multiply_xyzw(
        quarter_turn_about_z_xyzw,
        quarter_turn_about_z_xyzw,
    )

    assert math.isclose(composed_quaternion[2], 1.0, abs_tol=1.0e-9)
    assert math.isclose(composed_quaternion[3], 0.0, abs_tol=1.0e-9)


def test_quaternion_conjugate_xyzw_inverts_unit_rotation() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )

    identity_quaternion = quaternion_multiply_xyzw(
        quarter_turn_about_z_xyzw,
        quaternion_conjugate_xyzw(quarter_turn_about_z_xyzw),
    )

    assert math.isclose(identity_quaternion[0], 0.0, abs_tol=1.0e-9)
    assert math.isclose(identity_quaternion[1], 0.0, abs_tol=1.0e-9)
    assert math.isclose(identity_quaternion[2], 0.0, abs_tol=1.0e-9)
    assert math.isclose(identity_quaternion[3], 1.0, abs_tol=1.0e-9)


def test_quaternion_to_rotation_matrix_rotates_about_z() -> None:
    quarter_turn_about_z_xyzw: tuple[float, float, float, float] = (
        0.0,
        0.0,
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    )
    rotation_matrix: tuple[tuple[float, float, float], ...] = (
        quaternion_to_rotation_matrix(quarter_turn_about_z_xyzw)
    )

    assert math.isclose(rotation_matrix[0][0], 0.0, abs_tol=1.0e-9)
    assert math.isclose(rotation_matrix[0][1], -1.0, abs_tol=1.0e-9)
    assert math.isclose(rotation_matrix[1][0], 1.0, abs_tol=1.0e-9)
    assert math.isclose(rotation_matrix[1][1], 0.0, abs_tol=1.0e-9)


def test_rotate_vector_applies_matrix() -> None:
    rotation_matrix: tuple[tuple[float, float, float], ...] = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    rotated_vector: tuple[float, float, float] = rotate_vector(
        rotation_matrix,
        (1.0, 0.0, 0.0),
    )

    assert rotated_vector == (0.0, 1.0, 0.0)


def test_transpose_matrix_swaps_axes() -> None:
    matrix: tuple[tuple[float, float, float], ...] = (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    )
    transposed_matrix: tuple[tuple[float, float, float], ...] = transpose_matrix(matrix)

    assert transposed_matrix == (
        (1.0, 4.0, 7.0),
        (2.0, 5.0, 8.0),
        (3.0, 6.0, 9.0),
    )


def test_parse_row_major_matrix3_accepts_valid_values() -> None:
    parsed_matrix: tuple[tuple[float, float, float], ...] | None = (
        parse_row_major_matrix3((1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0))
    )

    assert parsed_matrix == (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    )


def test_parse_row_major_matrix3_rejects_invalid_length() -> None:
    parsed_matrix: tuple[tuple[float, float, float], ...] | None = (
        parse_row_major_matrix3((1.0, 2.0))
    )

    assert parsed_matrix is None


def test_parse_row_major_matrix3_rejects_non_finite_values() -> None:
    parsed_matrix: tuple[tuple[float, float, float], ...] | None = (
        parse_row_major_matrix3((1.0, 2.0, 3.0, 4.0, math.nan, 6.0, 7.0, 8.0, 9.0))
    )

    assert parsed_matrix is None


def test_parse_linear_covariance_3x3_extracts_leading_block() -> None:
    covariance_values: tuple[float, ...] = (
        1.0,
        2.0,
        3.0,
        0.0,
        0.0,
        0.0,
        4.0,
        5.0,
        6.0,
        0.0,
        0.0,
        0.0,
        7.0,
        8.0,
        9.0,
        0.0,
        0.0,
        0.0,
    ) + (0.0,) * 18
    parsed_matrix: tuple[tuple[float, float, float], ...] | None = (
        parse_linear_covariance_3x3(covariance_values)
    )

    assert parsed_matrix == (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    )


def test_parse_linear_covariance_3x3_rejects_invalid_length() -> None:
    parsed_matrix: tuple[tuple[float, float, float], ...] | None = (
        parse_linear_covariance_3x3((1.0,) * 35)
    )

    assert parsed_matrix is None


def test_flatten_matrix3_row_major_round_trips_shape() -> None:
    matrix: tuple[tuple[float, float, float], ...] = (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    )
    flattened_values: list[float] = flatten_matrix3_row_major(matrix)

    assert flattened_values == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]


def test_embed_linear_covariance_3x3_writes_leading_block() -> None:
    matrix: tuple[tuple[float, float, float], ...] = (
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
        (7.0, 8.0, 9.0),
    )
    embedded_values: list[float] = embed_linear_covariance_3x3(matrix)

    assert len(embedded_values) == 36
    assert embedded_values[0:3] == [1.0, 2.0, 3.0]
    assert embedded_values[6:9] == [4.0, 5.0, 6.0]
    assert embedded_values[12:15] == [7.0, 8.0, 9.0]


def test_rotate_covariance_swaps_axes() -> None:
    rotation_matrix: tuple[tuple[float, float, float], ...] = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    covariance_matrix: tuple[tuple[float, float, float], ...] = (
        (4.0, 0.0, 0.0),
        (0.0, 9.0, 0.0),
        (0.0, 0.0, 16.0),
    )
    rotated_covariance: tuple[tuple[float, float, float], ...] = rotate_covariance(
        rotation_matrix,
        covariance_matrix,
    )

    assert rotated_covariance == (
        (9.0, 0.0, 0.0),
        (0.0, 4.0, 0.0),
        (0.0, 0.0, 16.0),
    )


def test_is_positive_semidefinite_diagonal_accepts_nonnegative_diagonal() -> None:
    covariance_matrix: tuple[tuple[float, float, float], ...] = (
        (1.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (0.0, 0.0, 3.0),
    )

    assert is_positive_semidefinite_diagonal(covariance_matrix) is True


def test_is_positive_semidefinite_diagonal_rejects_negative_diagonal() -> None:
    covariance_matrix: tuple[tuple[float, float, float], ...] = (
        (-1.0, 0.0, 0.0),
        (0.0, 2.0, 0.0),
        (0.0, 0.0, 3.0),
    )

    assert is_positive_semidefinite_diagonal(covariance_matrix) is False
