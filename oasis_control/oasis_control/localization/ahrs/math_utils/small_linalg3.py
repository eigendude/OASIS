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

from typing import List
from typing import Sequence


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    """Return dot product of two vectors"""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def skew(v: Sequence[float]) -> List[List[float]]:
    """Return the 3x3 skew-symmetric matrix [v]_×"""
    return [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]


def matvec3(A: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    """Return A * v for 3x3 A and 3x1 v"""
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def matmul3(
    A: Sequence[Sequence[float]], B: Sequence[Sequence[float]]
) -> List[List[float]]:
    """Return A * B for 3x3 matrices"""
    return [
        [
            A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
            A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
            A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2],
        ],
        [
            A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
            A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
            A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2],
        ],
        [
            A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
            A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
            A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2],
        ],
    ]


def transpose3(A: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return transpose of a 3x3 matrix"""
    return [
        [A[0][0], A[1][0], A[2][0]],
        [A[0][1], A[1][1], A[2][1]],
        [A[0][2], A[1][2], A[2][2]],
    ]


def scale_mat(A: Sequence[Sequence[float]], scale: float) -> List[List[float]]:
    """Return scaled 3x3 matrix"""
    return [
        [A[0][0] * scale, A[0][1] * scale, A[0][2] * scale],
        [A[1][0] * scale, A[1][1] * scale, A[1][2] * scale],
        [A[2][0] * scale, A[2][1] * scale, A[2][2] * scale],
    ]
