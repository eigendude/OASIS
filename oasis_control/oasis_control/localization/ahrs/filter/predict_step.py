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

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra
from oasis_control.localization.ahrs.models.process_model import ProcessModel
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.covariance import AhrsCovariance
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


class PredictStep:
    """Prediction step for the AHRS EKF.

    Responsibility:
        Document the propagation of the mean state and covariance using the
        process model and first-order discretization.

    Purpose:
        Propagate the mean state and covariance forward in time using the
        continuous-time process model and first-order discretization.

    Inputs/outputs:
        - Inputs: AhrsState, AhrsCovariance, time step dt_ns.
        - Outputs: propagated state and covariance.

    Dependencies:
        - Depends on ProcessModel, StateMapping, and LinearAlgebra.

    Public API (to be implemented):
        - propagate(state, covariance, dt_ns)
        - compute_jacobians(state)
        - discretize(A, G, Q_c, dt_ns)

    Data contract:
        - state is an AhrsState instance.
        - covariance P is N x N with N from StateMapping.
        - dt_ns is a positive integer nanoseconds time step.
        - Canonical process noise ordering (q = 39):
          [w_v, w_ω, w_bg, w_ba, w_A, w_BI, w_BM, w_g, w_m]
        - Ordering is canonical and must never change.
        - Block dimensions:
          w_v(3), w_ω(3), w_bg(3), w_ba(3), w_A(9),
          w_BI(6), w_BM(6), w_g(3), w_m(3)
        - Q_c has shape (q, q).
        - G has shape (N, q) with N = StateMapping.dimension().

    Frames and units:
        - dt_ns in nanoseconds.
        - State units follow Units.

    Determinism and edge cases:
        - Prediction uses deterministic ordering and must be applied before
          any measurement updates at the same timestamp.
        - IMU and magnetometer samples are not process inputs.
        - Random-walk mean states are held constant during propagation.
        - dt_ns <= 0 should result in a no-op or a rejected step.

    Equations:
        First-order discretization:
            Define dt_sec := dt_ns * 1e-9 (deterministic scale; not used
            for keying).
            F ≈ I + Adt_sec
            Q ≈ G Q_c Gᵀ dt_sec

        Mean state propagation (zero-mean noise, no sampled injection):
            ṗ = v
            v̇ := 0
            q̇ = 0.5 * Ω(ω) * q
            ω̇ := 0
            b_g, b_a, A_a, T_BI, T_BM, g_W, m_W hold mean constant

        Continuous-time process noise (covariance only):
            v̇ = w_v, ω̇ = w_ω with E[w_v] = 0, E[w_ω] = 0

    Numerical stability notes:
        - Symmetrize covariance after propagation.
        - Ensure Q remains SPD.

    Suggested unit tests:
        - dt_ns=0 leaves state unchanged.
        - F and Q shapes are consistent with StateMapping.
    """

    @staticmethod
    def propagate(
        state: AhrsState,
        covariance: AhrsCovariance,
        dt_ns: int,
        Q_c: Sequence[Sequence[float]],
    ) -> tuple[AhrsState, AhrsCovariance]:
        """Propagate the mean state and covariance by dt_ns."""
        if not _is_int(dt_ns):
            raise ValueError("dt_ns must be int nanoseconds")
        if dt_ns < 0:
            raise ValueError("dt_ns must be non-negative")
        if dt_ns == 0:
            return (state, covariance)

        size: int = StateMapping.dimension()
        _assert_square("P", covariance.P, size)

        A: List[List[float]] = ProcessModel.linearize(state)
        G: List[List[float]] = ProcessModel.noise_jacobian(state)
        F: List[List[float]]
        Q: List[List[float]]
        F, Q = ProcessModel.discretize(A, G, Q_c, dt_ns)
        _assert_square("F", F, size)
        _assert_square("Q", Q, size)

        P: List[List[float]] = [list(row) for row in covariance.P]
        FP: List[List[float]] = _matmul(F, P)
        FPFt: List[List[float]] = _matmul(FP, _transpose(F))
        P_pred: List[List[float]] = _add_mat(FPFt, Q)
        P_sym: List[List[float]] = LinearAlgebra.symmetrize(P_pred)

        propagated: AhrsState = ProcessModel.propagate_mean(state, dt_ns)
        return (propagated, AhrsCovariance.from_matrix(P_sym))


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _assert_square(name: str, mat: Sequence[Sequence[float]], size: int) -> None:
    """Validate a square matrix size."""
    if len(mat) != size or any(len(row) != size for row in mat):
        raise ValueError(f"{name} must be {size}x{size}")


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
