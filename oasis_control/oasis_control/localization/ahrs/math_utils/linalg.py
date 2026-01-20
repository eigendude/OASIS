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
from typing import List
from typing import Sequence


class LinearAlgebra:
    """Linear-algebra helpers for AHRS covariance management.

    Responsibility:
        Provide backend-agnostic helpers for symmetric matrices, SPD checks,
        and numerical conditioning needed by the filter and models.

    Purpose:
        Define the operations the AHRS needs for covariance symmetrization,
        SPD checking, and linear solves without enforcing a numpy backend.

    Inputs/outputs:
        - Covariances are full matrices, never diagonalized.
        - Symmetrization is explicit: P <- 0.5 * (P + P^T).

    Dependencies:
        - Used by covariance, update_step, and noise_adaptation modules.

    Public API (to be implemented):
        - symmetrize(P)
        - is_spd(P)
        - clamp_spd(P, min_eig, max_eig)
        - solve_spd(A, b)
        - cholesky(A)

    Data contract:
        - P, A are square matrices of shape (n, n).
        - b is a vector of shape (n,) or matrix of shape (n, m).

    Frames and units:
        - Covariance units follow the state units in Units.
        - No implicit unit conversions are performed.

    Determinism and edge cases:
        - Deterministic behavior is required for SPD checks and clamping so
          repeated runs produce identical decisions about accepting updates.
        - is_spd must be deterministic for nearly singular matrices.
        - clamp_spd must preserve symmetry and clamp eigenvalues.

    Equations:
        Symmetrization:
            P_sym = 0.5 * (P + Pᵀ)

    Numerical stability notes:
        - clamp_spd should avoid negative eigenvalues caused by round-off.
        - solve_spd should prefer Cholesky for SPD matrices.

    Suggested unit tests:
        - symmetrize returns symmetric result for asymmetric input.
        - is_spd rejects matrices with non-positive eigenvalues.
        - clamp_spd keeps eigenvalues within [min_eig, max_eig].
    """

    @staticmethod
    def symmetrize(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
        LinearAlgebra._validate_square(matrix, "symmetrize")
        size: int = len(matrix)
        result: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        for i in range(size):
            for j in range(size):
                result[i][j] = 0.5 * (matrix[i][j] + matrix[j][i])
        return result

    @staticmethod
    def is_spd(matrix: Sequence[Sequence[float]]) -> bool:
        try:
            LinearAlgebra.cholesky(matrix)
        except ValueError:
            return False
        return True

    @staticmethod
    def cholesky(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
        LinearAlgebra._validate_square(matrix, "cholesky")
        size: int = len(matrix)
        lower: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        for i in range(size):
            for j in range(i + 1):
                acc: float = matrix[i][j]
                for k in range(j):
                    acc -= lower[i][k] * lower[j][k]
                if i == j:
                    if acc <= 0.0 or math.isnan(acc):
                        raise ValueError("Matrix is not SPD")
                    lower[i][j] = math.sqrt(acc)
                else:
                    if lower[j][j] == 0.0:
                        raise ValueError("Matrix is not SPD")
                    lower[i][j] = acc / lower[j][j]
        return lower

    @staticmethod
    def solve_spd(
        matrix: Sequence[Sequence[float]],
        b: Sequence[float] | Sequence[Sequence[float]],
    ) -> List[float] | List[List[float]]:
        lower: List[List[float]] = LinearAlgebra.cholesky(matrix)
        if LinearAlgebra._is_vector(b):
            b_vec: Sequence[float] = b  # type: ignore[assignment]
            y: List[float] = LinearAlgebra._forward_sub(lower, b_vec)
            return LinearAlgebra._backward_sub_transpose(lower, y)
        b_mat: Sequence[Sequence[float]] = b  # type: ignore[assignment]
        LinearAlgebra._validate_rows(b_mat, len(lower), "solve_spd")
        cols: int = len(b_mat[0])
        result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(len(lower))]
        for col in range(cols):
            column: List[float] = [row[col] for row in b_mat]
            y_col: List[float] = LinearAlgebra._forward_sub(lower, column)
            x_col: List[float] = LinearAlgebra._backward_sub_transpose(lower, y_col)
            for row in range(len(lower)):
                result[row][col] = x_col[row]
        return result

    @staticmethod
    def clamp_spd(
        matrix: Sequence[Sequence[float]],
        min_eig: float,
        max_eig: float,
    ) -> List[List[float]]:
        LinearAlgebra._validate_square(matrix, "clamp_spd")
        if min_eig <= 0.0:
            raise ValueError("min_eig must be positive")
        if max_eig < min_eig:
            raise ValueError("max_eig must be >= min_eig")
        size: int = len(matrix)
        a: List[List[float]] = LinearAlgebra.symmetrize(matrix)
        v: List[List[float]] = LinearAlgebra._identity(size)
        max_sweeps: int = 20
        tol: float = 1e-12
        for _ in range(max_sweeps):
            off_norm: float = LinearAlgebra._off_diag_norm(a)
            if off_norm <= tol:
                break
            for p in range(size - 1):
                for q in range(p + 1, size):
                    apq: float = a[p][q]
                    if abs(apq) <= tol:
                        continue
                    tau: float = (a[q][q] - a[p][p]) / (2.0 * apq)
                    t_sign: float = 1.0 if tau >= 0.0 else -1.0
                    t: float = t_sign / (abs(tau) + math.sqrt(1.0 + tau * tau))
                    c: float = 1.0 / math.sqrt(1.0 + t * t)
                    s: float = t * c
                    for k in range(size):
                        if k == p or k == q:
                            continue
                        akp: float = a[k][p]
                        akq: float = a[k][q]
                        a[k][p] = c * akp - s * akq
                        a[p][k] = a[k][p]
                        a[k][q] = s * akp + c * akq
                        a[q][k] = a[k][q]
                    app: float = a[p][p]
                    aqq: float = a[q][q]
                    a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
                    a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
                    a[p][q] = 0.0
                    a[q][p] = 0.0
                    for k in range(size):
                        vkp: float = v[k][p]
                        vkq: float = v[k][q]
                        v[k][p] = c * vkp - s * vkq
                        v[k][q] = s * vkp + c * vkq
        lambdas: List[float] = [a[i][i] for i in range(size)]
        lambdas = [min(max_eig, max(min_eig, lam)) for lam in lambdas]
        result: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        for i in range(size):
            for j in range(size):
                acc: float = 0.0
                for k in range(size):
                    acc += v[i][k] * lambdas[k] * v[j][k]
                result[i][j] = acc
        return LinearAlgebra.symmetrize(result)

    @staticmethod
    def _validate_square(
        matrix: Sequence[Sequence[float]],
        name: str,
    ) -> None:
        if not matrix:
            raise ValueError(f"{name} requires a non-empty matrix")
        size: int = len(matrix)
        for row in matrix:
            if len(row) != size:
                raise ValueError(f"{name} requires a square matrix")

    @staticmethod
    def _validate_rows(
        matrix: Sequence[Sequence[float]],
        rows: int,
        name: str,
    ) -> None:
        if not matrix:
            raise ValueError(f"{name} requires a non-empty matrix")
        if len(matrix) != rows:
            raise ValueError(f"{name} requires {rows} rows")
        cols: int = len(matrix[0])
        for row in matrix:
            if len(row) != cols:
                raise ValueError(f"{name} requires a rectangular matrix")

    @staticmethod
    def _is_vector(b: Sequence[float] | Sequence[Sequence[float]]) -> bool:
        if not b:
            raise ValueError("solve_spd requires a non-empty right-hand side")
        return not isinstance(b[0], Sequence)  # type: ignore[index]

    @staticmethod
    def _forward_sub(lower: Sequence[Sequence[float]], b: Sequence[float]) -> List[float]:
        size: int = len(lower)
        if len(b) != size:
            raise ValueError("solve_spd dimension mismatch")
        y: List[float] = [0.0 for _ in range(size)]
        for i in range(size):
            acc: float = b[i]
            for k in range(i):
                acc -= lower[i][k] * y[k]
            if lower[i][i] == 0.0:
                raise ValueError("Matrix is not SPD")
            y[i] = acc / lower[i][i]
        return y

    @staticmethod
    def _backward_sub_transpose(
        lower: Sequence[Sequence[float]],
        y: Sequence[float],
    ) -> List[float]:
        size: int = len(lower)
        x: List[float] = [0.0 for _ in range(size)]
        for i in range(size - 1, -1, -1):
            acc: float = y[i]
            for k in range(i + 1, size):
                acc -= lower[k][i] * x[k]
            if lower[i][i] == 0.0:
                raise ValueError("Matrix is not SPD")
            x[i] = acc / lower[i][i]
        return x

    @staticmethod
    def _identity(size: int) -> List[List[float]]:
        return [[1.0 if i == j else 0.0 for j in range(size)] for i in range(size)]

    @staticmethod
    def _off_diag_norm(matrix: Sequence[Sequence[float]]) -> float:
        size: int = len(matrix)
        acc: float = 0.0
        for i in range(size):
            for j in range(size):
                if i != j:
                    acc += matrix[i][j] * matrix[i][j]
        return math.sqrt(acc)
