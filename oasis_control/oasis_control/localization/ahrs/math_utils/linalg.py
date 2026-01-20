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

from math import sqrt
from typing import List, Sequence


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
    def symmetrize(P: Sequence[Sequence[float]]) -> List[List[float]]:
        """Return the symmetric part 0.5 * (P + P^T)."""
        n: int = len(P)
        if any(len(row) != n for row in P):
            raise ValueError("Matrix must be square")
        symmetric: List[List[float]] = [[0.0 for _ in range(n)] for _ in range(n)]
        for i in range(n):
            for j in range(n):
                # 0.5 scales the sum of mirrored entries to the average
                symmetric[i][j] = 0.5 * (P[i][j] + P[j][i])
        return symmetric

    @staticmethod
    def cholesky(A: Sequence[Sequence[float]]) -> List[List[float]]:
        """Compute the lower-triangular Cholesky factor of SPD matrix A."""
        n: int = len(A)
        if any(len(row) != n for row in A):
            raise ValueError("Matrix must be square")
        L: List[List[float]] = [[0.0 for _ in range(n)] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1):
                acc: float = 0.0
                for k in range(j):
                    acc += L[i][k] * L[j][k]
                if i == j:
                    diag: float = A[i][i] - acc
                    if diag <= 0.0:
                        raise ValueError("Matrix is not SPD")
                    L[i][j] = sqrt(diag)
                else:
                    if L[j][j] == 0.0:
                        raise ValueError("Matrix is not SPD")
                    L[i][j] = (A[i][j] - acc) / L[j][j]
        return L

    @staticmethod
    def is_spd(P: Sequence[Sequence[float]]) -> bool:
        """Return True if P is symmetric positive definite."""
        try:
            LinearAlgebra.cholesky(P)
        except ValueError:
            return False
        return True

    @staticmethod
    def solve_spd(
        A: Sequence[Sequence[float]],
        b: Sequence[float] | Sequence[Sequence[float]],
    ) -> List[float] | List[List[float]]:
        """Solve A x = b for SPD A using Cholesky factorization."""
        L: List[List[float]] = LinearAlgebra.cholesky(A)
        n: int = len(L)
        if isinstance(b[0], (list, tuple)):
            b_mat: Sequence[Sequence[float]] = b  # type: ignore[assignment]
            if any(len(row) != n for row in b_mat):
                raise ValueError("Right-hand matrix has wrong shape")
            m: int = len(b_mat[0])
            for row in b_mat:
                if len(row) != m:
                    raise ValueError("Right-hand matrix has ragged rows")
            y: List[List[float]] = [[0.0 for _ in range(m)] for _ in range(n)]
            for i in range(n):
                for j in range(m):
                    acc: float = 0.0
                    for k in range(i):
                        acc += L[i][k] * y[k][j]
                    y[i][j] = (b_mat[i][j] - acc) / L[i][i]
            x: List[List[float]] = [[0.0 for _ in range(m)] for _ in range(n)]
            for i in range(n - 1, -1, -1):
                for j in range(m):
                    acc = 0.0
                    for k in range(i + 1, n):
                        acc += L[k][i] * x[k][j]
                    x[i][j] = (y[i][j] - acc) / L[i][i]
            return x
        b_vec: Sequence[float] = b  # type: ignore[assignment]
        if len(b_vec) != n:
            raise ValueError("Right-hand vector has wrong length")
        y_vec: List[float] = [0.0 for _ in range(n)]
        for i in range(n):
            acc = 0.0
            for k in range(i):
                acc += L[i][k] * y_vec[k]
            y_vec[i] = (b_vec[i] - acc) / L[i][i]
        x_vec: List[float] = [0.0 for _ in range(n)]
        for i in range(n - 1, -1, -1):
            acc = 0.0
            for k in range(i + 1, n):
                acc += L[k][i] * x_vec[k]
            x_vec[i] = (y_vec[i] - acc) / L[i][i]
        return x_vec

    @staticmethod
    def clamp_spd(
        P: Sequence[Sequence[float]],
        min_eig: float,
        max_eig: float,
    ) -> List[List[float]]:
        """Clamp eigenvalues of a symmetric matrix using Jacobi iterations."""
        if min_eig <= 0.0:
            raise ValueError("min_eig must be positive")
        if max_eig < min_eig:
            raise ValueError("max_eig must be >= min_eig")
        n: int = len(P)
        if any(len(row) != n for row in P):
            raise ValueError("Matrix must be square")
        A: List[List[float]] = [list(row) for row in P]
        A = LinearAlgebra.symmetrize(A)
        V: List[List[float]] = [[0.0 for _ in range(n)] for _ in range(n)]
        for i in range(n):
            V[i][i] = 1.0
        # Bound sweep count for deterministic, finite iteration count
        max_sweeps: int = 50
        # Squared off-diagonal norm threshold for convergence
        tol: float = 1e-12
        for _ in range(max_sweeps):
            off_norm: float = 0.0
            for i in range(n):
                for j in range(i + 1, n):
                    off_norm += A[i][j] * A[i][j]
            if off_norm < tol:
                break
            for p in range(n - 1):
                for q in range(p + 1, n):
                    apq: float = A[p][q]
                    if abs(apq) < tol:
                        continue
                    app: float = A[p][p]
                    aqq: float = A[q][q]
                    # tau is (aqq - app) / (2 * apq) for Jacobi rotation
                    tau: float = (aqq - app) / (2.0 * apq)
                    if tau >= 0.0:
                        t: float = 1.0 / (tau + sqrt(1.0 + tau * tau))
                    else:
                        t = -1.0 / (-tau + sqrt(1.0 + tau * tau))
                    # c and s form the Givens rotation from t = s / c
                    c: float = 1.0 / sqrt(1.0 + t * t)
                    s: float = t * c
                    for k in range(n):
                        if k != p and k != q:
                            akp: float = A[k][p]
                            akq: float = A[k][q]
                            A[k][p] = c * akp - s * akq
                            A[p][k] = A[k][p]
                            A[k][q] = c * akq + s * akp
                            A[q][k] = A[k][q]
                    A[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
                    A[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
                    A[p][q] = 0.0
                    A[q][p] = 0.0
                    for k in range(n):
                        vkp: float = V[k][p]
                        vkq: float = V[k][q]
                        V[k][p] = c * vkp - s * vkq
                        V[k][q] = s * vkp + c * vkq
        clamped: List[float] = [0.0 for _ in range(n)]
        for i in range(n):
            eig: float = A[i][i]
            if eig < min_eig:
                eig = min_eig
            elif eig > max_eig:
                eig = max_eig
            clamped[i] = eig
        P_clamped: List[List[float]] = [[0.0 for _ in range(n)] for _ in range(n)]
        for i in range(n):
            for j in range(n):
                acc = 0.0
                for k in range(n):
                    acc += V[i][k] * clamped[k] * V[j][k]
                P_clamped[i][j] = acc
        return LinearAlgebra.symmetrize(P_clamped)
