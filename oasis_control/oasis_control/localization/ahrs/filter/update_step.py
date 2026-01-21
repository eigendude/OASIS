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
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance


@dataclass(frozen=True, slots=True)
class UpdateReport:
    """Report for EKF update acceptance and innovation metrics.

    Data contract:
        accepted:
            True when the update was applied
        reason:
            Deterministic rejection reason string when accepted is False
        innovation_mahalanobis2:
            νᵀ S⁻¹ ν scalar, dimensionless and >= 0, None on rejection
    """

    accepted: bool
    reason: str
    innovation_mahalanobis2: float | None


class UpdateStep:
    """Measurement update step for the AHRS EKF.

    Responsibility:
        Document the generic EKF update equations, Joseph-form covariance
        update, and rejection policies for invalid innovation covariance.

    Purpose:
        Apply a measurement update using the EKF equations with a Joseph-form
        covariance update and explicit rejection rules.

    Inputs/outputs:
        - Inputs: P (N x N), H (m x N), R (m x m), nu (m,), z, z_hat.
        - Outputs: updated state correction and covariance, plus UpdateReport.

    Dependencies:
        - Uses Statistics for innovation metrics and LinearAlgebra for SPD
          checks.

    Public API (to be implemented):
        - compute_innovation(H, P, R, nu)
        - compute_kalman_gain(P, H, S)
        - apply_update(state, covariance, K, nu)
        - update_report(...)

    Data contract:
        - P: N x N covariance matrix.
        - H: m x N measurement Jacobian.
        - R: m x m measurement covariance.
        - nu: length-m innovation residual.
        - z and z_hat: length-m measurement and prediction.

    Frames and units:
        - gyro/accel: {I}
        - mag: {M}
        - zupt: {W}
        - no_turn: {B}
        - Covariance units align with Units.

    Determinism and edge cases:
        - Updates are deterministic and applied in the prescribed order.
        - If S is not SPD or solving fails, reject the update and record it
          in UpdateReport.
        - On rejection, state and covariance remain unchanged (no partial
          update).
        - Symmetrize P after applying the Joseph-form update.

    Equations:
        Innovation and gain:
            S = H P Hᵀ + R
            K = P Hᵀ S⁻¹
            δx = K ν

        Joseph-form covariance update:
            P <- (I - K H) P (I - K H)ᵀ + K R Kᵀ

    Numerical stability notes:
        - Use SPD solvers for S to avoid explicit inversion.
        - Symmetrize P after the update.

    Suggested unit tests:
        - Reject updates when S is not SPD.
        - Joseph-form update preserves symmetry.
    """

    @staticmethod
    def compute_innovation(
        H: Sequence[Sequence[float]],
        P: Sequence[Sequence[float]],
        R: Sequence[Sequence[float]],
        nu: Sequence[float],
    ) -> List[List[float]]:
        """Return S = H P Hᵀ + R for innovation covariance."""
        _validate_update_inputs(P, H, R, nu)
        HP: List[List[float]] = _matmul(H, P)
        S_hat: List[List[float]] = _matmul(HP, _transpose(H))
        S: List[List[float]] = _add_mat(S_hat, R)
        _assert_square("S", S, len(R))
        _assert_finite_matrix("S", S)
        return S

    @staticmethod
    def compute_kalman_gain(
        P: Sequence[Sequence[float]],
        H: Sequence[Sequence[float]],
        S: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        """Return the Kalman gain K for the update."""
        _assert_square("P", P, len(P))
        if len(H) == 0:
            raise ValueError("H must have at least one row")
        if len(H[0]) != len(P):
            raise ValueError("H must have N columns")
        if len(S) != len(H) or any(len(row) != len(H) for row in S):
            raise ValueError("S must be mxm with m = len(H)")
        _assert_finite_matrix("P", P)
        _assert_finite_matrix("H", H)
        _assert_finite_matrix("S", S)
        if not LinearAlgebra.is_spd(S):
            raise ValueError("S is not SPD")

        Ht: List[List[float]] = _transpose(H)
        PHt: List[List[float]] = _matmul(P, Ht)
        PHt_T: List[List[float]] = _transpose(PHt)
        try:
            X: List[List[float]] = _solve_spd_matrix(S, PHt_T)
        except ValueError as exc:
            raise ValueError("solve_spd failed") from exc
        K: List[List[float]] = _transpose(X)
        return K

    @staticmethod
    def apply_update(
        state: AhrsState,
        covariance: AhrsCovariance,
        K: Sequence[Sequence[float]],
        nu: Sequence[float],
    ) -> tuple[AhrsState, AhrsCovariance]:
        """Apply δx = Kν to the state and return updated state/covariance."""
        if len(K) == 0:
            raise ValueError("K must have at least one row")
        m: int = len(nu)
        row: Sequence[float]
        for row in K:
            if len(row) != m:
                raise ValueError("K must have m columns matching nu")
        _assert_finite_matrix("K", K)
        _assert_finite_vector("nu", nu)
        delta_x: List[float] = _matvec(K, nu)
        updated_state: AhrsState = state.apply_error(delta_x)
        return (updated_state, covariance)

    @staticmethod
    def update(
        state: AhrsState,
        covariance: AhrsCovariance,
        H: Sequence[Sequence[float]],
        R: Sequence[Sequence[float]],
        nu: Sequence[float],
    ) -> tuple[AhrsState, AhrsCovariance, UpdateReport]:
        """Return updated state, covariance, and report for the EKF update."""
        try:
            S: List[List[float]] = UpdateStep.compute_innovation(
                H,
                covariance.P,
                R,
                nu,
            )
            if not LinearAlgebra.is_spd(S):
                raise ValueError("S is not SPD")
            K: List[List[float]] = UpdateStep.compute_kalman_gain(covariance.P, H, S)
            P_new: List[List[float]] = _joseph_update(covariance.P, K, H, R)
            P_new = LinearAlgebra.symmetrize(P_new)
            cov_new: AhrsCovariance = AhrsCovariance.from_matrix(P_new)
            updated_state: AhrsState
            updated_state, cov_new = UpdateStep.apply_update(state, cov_new, K, nu)
            mahalanobis2: float = _mahalanobis2(S, nu)
            report: UpdateReport = UpdateReport(
                accepted=True,
                reason="",
                innovation_mahalanobis2=mahalanobis2,
            )
            return (updated_state, cov_new, report)
        except ValueError as exc:
            report_reject: UpdateReport = UpdateReport(
                accepted=False,
                reason=str(exc),
                innovation_mahalanobis2=None,
            )
            return (state, covariance, report_reject)


def _validate_update_inputs(
    P: Sequence[Sequence[float]],
    H: Sequence[Sequence[float]],
    R: Sequence[Sequence[float]],
    nu: Sequence[float],
) -> None:
    """Validate shapes and finiteness for update inputs."""
    if len(P) == 0 or any(len(row) != len(P) for row in P):
        raise ValueError("P must be NxN")
    if len(H) == 0:
        raise ValueError("H must have at least one row")
    n: int = len(P)
    if any(len(row) != n for row in H):
        raise ValueError("H must have N columns")
    m: int = len(H)
    if len(R) != m or any(len(row) != m for row in R):
        raise ValueError("R must be mxm with m = len(H)")
    if len(nu) != m:
        raise ValueError("nu must have length m = len(H)")
    _assert_finite_matrix("P", P)
    _assert_finite_matrix("H", H)
    _assert_finite_matrix("R", R)
    _assert_finite_vector("nu", nu)


def _assert_square(name: str, mat: Sequence[Sequence[float]], size: int) -> None:
    """Validate a square matrix size."""
    if len(mat) != size or any(len(row) != size for row in mat):
        raise ValueError(f"{name} must be {size}x{size}")


def _assert_finite_vector(name: str, vec: Sequence[float]) -> None:
    """Validate vector entries are finite."""
    value: float
    for value in vec:
        if not math.isfinite(value):
            raise ValueError(f"{name} contains non-finite value")


def _assert_finite_matrix(name: str, mat: Sequence[Sequence[float]]) -> None:
    """Validate matrix entries are finite."""
    row: Sequence[float]
    for row in mat:
        value: float
        for value in row:
            if not math.isfinite(value):
                raise ValueError(f"{name} contains non-finite value")


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


def _matvec(
    A: Sequence[Sequence[float]],
    b: Sequence[float],
) -> List[float]:
    """Multiply matrix A by vector b."""
    rows: int = len(A)
    cols: int = len(A[0])
    if len(b) != cols:
        raise ValueError("Vector length does not match matrix columns")
    result: List[float] = [0.0 for _ in range(rows)]
    i: int
    for i in range(rows):
        acc: float = 0.0
        j: int
        for j in range(cols):
            acc += A[i][j] * b[j]
        result[i] = acc
    return result


def _add_mat(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Add two matrices."""
    rows: int = len(A)
    cols: int = len(A[0])
    result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(rows)]
    i: int
    for i in range(rows):
        j: int
        for j in range(cols):
            result[i][j] = A[i][j] + B[i][j]
    return result


def _joseph_update(
    P: Sequence[Sequence[float]],
    K: Sequence[Sequence[float]],
    H: Sequence[Sequence[float]],
    R: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Return Joseph-form covariance update."""
    _assert_square("P", P, len(P))
    if len(K) != len(P):
        raise ValueError("K must have N rows")
    if len(H) == 0:
        raise ValueError("H must have at least one row")
    if len(H[0]) != len(P):
        raise ValueError("H must have N columns")
    if len(R) != len(H) or any(len(row) != len(H) for row in R):
        raise ValueError("R must be mxm with m = len(H)")
    if any(len(row) != len(H) for row in K):
        raise ValueError("K must have m columns matching H")

    KH: List[List[float]] = _matmul(K, H)
    I: List[List[float]] = _identity(len(P))
    I_minus_KH: List[List[float]] = _sub_mat(I, KH)
    left: List[List[float]] = _matmul(I_minus_KH, P)
    left_right: List[List[float]] = _matmul(left, _transpose(I_minus_KH))
    KR: List[List[float]] = _matmul(K, R)
    KRKt: List[List[float]] = _matmul(KR, _transpose(K))
    return _add_mat(left_right, KRKt)


def _identity(size: int) -> List[List[float]]:
    """Return identity matrix."""
    result: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
    i: int
    for i in range(size):
        result[i][i] = 1.0
    return result


def _sub_mat(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Subtract two matrices."""
    rows: int = len(A)
    cols: int = len(A[0])
    result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(rows)]
    i: int
    for i in range(rows):
        j: int
        for j in range(cols):
            result[i][j] = A[i][j] - B[i][j]
    return result


def _mahalanobis2(S: Sequence[Sequence[float]], nu: Sequence[float]) -> float:
    """Return νᵀ S⁻¹ ν using SPD solve."""
    if not LinearAlgebra.is_spd(S):
        raise ValueError("S is not SPD")
    try:
        solved: List[float] = LinearAlgebra.solve_spd(S, nu)
    except ValueError as exc:
        raise ValueError("solve_spd failed") from exc
    acc: float = 0.0
    idx: int
    for idx in range(len(nu)):
        acc += nu[idx] * solved[idx]
    if not math.isfinite(acc):
        raise ValueError("mahalanobis2 is not finite")
    if acc < 0.0:
        raise ValueError("mahalanobis2 must be >= 0")
    return acc


def _solve_spd_matrix(
    S: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Solve S X = B for SPD S using column-wise solves."""
    m: int = len(S)
    if len(B) != m:
        raise ValueError("Right-hand matrix has wrong shape")
    if any(len(row) != len(B[0]) for row in B):
        raise ValueError("Right-hand matrix has ragged rows")
    n: int = len(B[0])
    X: List[List[float]] = [[0.0 for _ in range(n)] for _ in range(m)]
    col: int
    for col in range(n):
        b_vec: List[float] = [B[row][col] for row in range(m)]
        x_vec: List[float] = LinearAlgebra.solve_spd(S, b_vec)
        row: int
        for row in range(m):
            X[row][col] = x_vec[row]
    return X
