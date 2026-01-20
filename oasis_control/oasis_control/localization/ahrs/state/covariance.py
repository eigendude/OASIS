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
from typing import List
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


@dataclass(frozen=True, slots=True)
class AhrsCovariance:
    """Covariance container and operations for the AHRS EKF.

    Responsibility:
        Store and document the full error-state covariance matrix used by the
        EKF, including symmetrization and mapping rules.

    Purpose:
        Maintain the error-state covariance matrix with explicit symmetrization
        and mapping rules between coordinate frames.

    Inputs/outputs:
        - P is an N x N full covariance matrix.
        - Jacobian-based output covariance uses Σ_y = J P J^T.

    Dependencies:
        - Uses StateMapping for ordering and LinearAlgebra for symmetrization.

    Public API:
        - symmetrize()
        - as_matrix()
        - from_matrix(P)
        - map_with_jacobian(J)
        - apply_adjoint(Ad_T, block_slice)

    Data contract:
        - P is an N x N matrix with N defined by StateMapping.
        - All covariance blocks are full and symmetric.

    Frames and units:
        - Covariance units are derived from Units for each state element.
        - SE(3) covariance mapping uses the adjoint of the transform.

    Determinism and edge cases:
        - Symmetrization must be applied deterministically after any update.
        - Always apply P <- 0.5 * (P + P^T) after updates.
        - Mapping with Jacobians must preserve symmetry.

    Equations:
        Output covariance for y = f(x):
            Σ_y = J P J^T

        SE(3) adjoint mapping:
            Σ_A = Ad_T * Σ_B * Ad_T^T

    Numerical stability notes:
        - Symmetrize after each update to mitigate round-off.
        - Avoid implicit diagonalization.

    Suggested unit tests:
        - symmetrize yields symmetric matrix.
        - map_with_jacobian matches manual multiplication.
    """

    P: List[List[float]]

    def as_matrix(self) -> List[List[float]]:
        """Return a deep copy of the covariance matrix."""
        return [list(row) for row in self.P]

    def symmetrize(self) -> AhrsCovariance:
        """Return a symmetrized copy of the covariance matrix."""
        sym: List[List[float]] = LinearAlgebra.symmetrize(self.P)
        return AhrsCovariance(P=sym)

    @staticmethod
    def from_matrix(P: Sequence[Sequence[float]]) -> AhrsCovariance:
        """Create a covariance from a raw matrix and symmetrize."""
        _assert_square(P, StateMapping.dimension())
        _assert_finite_matrix(P)
        copied: List[List[float]] = [list(row) for row in P]
        sym: List[List[float]] = LinearAlgebra.symmetrize(copied)
        return AhrsCovariance(P=sym)

    def map_with_jacobian(self, J: Sequence[Sequence[float]]) -> List[List[float]]:
        """Return Σ = J P Jᵀ for a Jacobian J."""
        _assert_jacobian_shape(J, StateMapping.dimension())
        _assert_finite_matrix(J)
        JP: List[List[float]] = _matmul(J, self.P)
        JT: List[List[float]] = _transpose(J)
        return _matmul(JP, JT)

    def apply_adjoint(
        self, Ad_T: Sequence[Sequence[float]], block_slice: slice
    ) -> AhrsCovariance:
        """Return a covariance with Ad_T applied to a 6x6 block."""
        _assert_square(Ad_T, 6)
        _assert_finite_matrix(Ad_T)
        if block_slice.start is None or block_slice.stop is None:
            raise ValueError("block_slice must have explicit bounds for 6x6 block")
        if block_slice.step not in (None, 1):
            raise ValueError("block_slice must have unit step for 6x6 block")
        if block_slice.stop - block_slice.start != 6:
            raise ValueError("block_slice must select exactly 6 elements")
        size: int = StateMapping.dimension()
        _assert_square(self.P, size)
        start: int = block_slice.start
        end: int = block_slice.stop
        block: List[List[float]] = [
            list(self.P[i][start:end]) for i in range(start, end)
        ]
        mapped: List[List[float]] = _map_covariance_block(Ad_T, block)
        updated: List[List[float]] = [list(row) for row in self.P]
        i: int
        for i in range(6):
            j: int
            for j in range(6):
                updated[start + i][start + j] = mapped[i][j]
                updated[start + j][start + i] = mapped[j][i]
        other_indices: List[int] = [
            idx for idx in range(size) if idx < start or idx >= end
        ]
        if other_indices:
            cross_block: List[List[float]] = [
                [self.P[start + row][col] for col in other_indices] for row in range(6)
            ]
            mapped_cross: List[List[float]] = _matmul(Ad_T, cross_block)
            mapped_cross_t: List[List[float]] = _transpose(mapped_cross)
            row: int
            for row in range(6):
                col_idx: int
                for col_idx, col in enumerate(other_indices):
                    updated[start + row][col] = mapped_cross[row][col_idx]
                    updated[col][start + row] = mapped_cross_t[col_idx][row]
        return AhrsCovariance(P=LinearAlgebra.symmetrize(updated))


def _assert_square(P: Sequence[Sequence[float]], size: int) -> None:
    """Validate matrix is square with specified size."""
    if len(P) != size or any(len(row) != size for row in P):
        raise ValueError(f"Matrix must be {size}x{size}")


def _assert_finite_matrix(P: Sequence[Sequence[float]]) -> None:
    """Validate matrix entries are finite."""
    row: Sequence[float]
    for row in P:
        value: float
        for value in row:
            if not math.isfinite(value):
                raise ValueError("Matrix contains non-finite value")


def _assert_jacobian_shape(J: Sequence[Sequence[float]], cols: int) -> None:
    """Validate Jacobian has consistent row sizes."""
    if len(J) == 0:
        raise ValueError("Jacobian must have at least one row")
    row: Sequence[float]
    for row in J:
        if len(row) != cols:
            raise ValueError("Jacobian has wrong shape")


def _transpose(A: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return the transpose of a matrix."""
    rows: int = len(A)
    cols: int = len(A[0])
    result: List[List[float]] = [[0.0 for _ in range(rows)] for _ in range(cols)]
    i: int
    for i in range(rows):
        j: int
        for j in range(cols):
            result[j][i] = A[i][j]
    return result


def _matmul(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Multiply two matrices."""
    rows: int = len(A)
    cols: int = len(B[0])
    inner: int = len(B)
    result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(rows)]
    i: int
    for i in range(rows):
        j: int
        for j in range(cols):
            acc: float = 0.0
            k: int
            for k in range(inner):
                acc += A[i][k] * B[k][j]
            result[i][j] = acc
    return result


def _map_covariance_block(
    Ad_T: Sequence[Sequence[float]],
    block: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Map a 6x6 covariance block with the adjoint."""
    temp: List[List[float]] = _matmul(Ad_T, block)
    Ad_T_t: List[List[float]] = _transpose(Ad_T)
    return _matmul(temp, Ad_T_t)
