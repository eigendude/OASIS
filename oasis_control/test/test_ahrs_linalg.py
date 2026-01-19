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

from oasis_control.localization.ahrs.ahrs_linalg import is_finite_seq
from oasis_control.localization.ahrs.ahrs_linalg import mahalanobis_d2_3
from oasis_control.localization.ahrs.ahrs_linalg import mat3_inv
from oasis_control.localization.ahrs.ahrs_linalg import mat3_solve
from oasis_control.localization.ahrs.ahrs_linalg import mat_mul
from oasis_control.localization.ahrs.ahrs_linalg import mat_transpose
from oasis_control.localization.ahrs.ahrs_linalg import project_to_symmetric_psd_3
from oasis_control.localization.ahrs.ahrs_linalg import symmetrize


def test_transpose_roundtrip_2x3() -> None:
    original: list[float] = [
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
    ]
    transposed: list[float] = mat_transpose(original, 2, 3)
    roundtrip: list[float] = mat_transpose(transposed, 3, 2)

    assert roundtrip == original


def test_mat_mul_identity_3x3() -> None:
    identity: list[float] = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    matrix: list[float] = [
        2.0,
        -1.0,
        3.0,
        0.5,
        4.0,
        2.0,
        -2.0,
        1.0,
        0.25,
    ]

    product: list[float] = mat_mul(matrix, 3, 3, identity, 3, 3)

    assert product == matrix


def test_mat3_inv_identity() -> None:
    identity: list[float] = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    inverted: list[float] = mat3_inv(identity)

    assert inverted == identity


def test_mat3_solve_known_system() -> None:
    matrix: list[float] = [
        2.0,
        0.0,
        0.0,
        0.0,
        3.0,
        0.0,
        0.0,
        0.0,
        4.0,
    ]
    b: list[float] = [2.0, 9.0, 8.0]

    solution: list[float] = mat3_solve(matrix, b)

    assert solution == [1.0, 3.0, 2.0]


def test_symmetrize_makes_symmetric() -> None:
    matrix: list[float] = [
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
    ]

    symmetric: list[float] = symmetrize(matrix, 3)

    assert symmetric[1] == symmetric[3]
    assert symmetric[2] == symmetric[6]
    assert symmetric[5] == symmetric[7]


def test_mahalanobis_d2_matches_manual() -> None:
    identity: list[float] = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    nu: list[float] = [1.0, 2.0, 3.0]

    d2: float = mahalanobis_d2_3(nu, identity)

    assert d2 == 14.0


def test_project_to_symmetric_psd_3_fallback_on_bad_matrix() -> None:
    matrix: list[float] = [
        1.0,
        2.0,
        0.0,
        2.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    projected: list[float] = project_to_symmetric_psd_3(matrix, diag_min=0.1)

    assert projected == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def test_is_finite_seq() -> None:
    assert is_finite_seq([0.0, 1.0, -2.5])
    assert not is_finite_seq([math.inf, 1.0])
    assert not is_finite_seq([math.nan, 0.0])
