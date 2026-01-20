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
from oasis_control.localization.ahrs.math_utils.quat import Quaternion
from oasis_control.localization.ahrs.state.ahrs_state import AhrsState
from oasis_control.localization.ahrs.state.state_mapping import StateMapping


class ProcessModel:
    """Continuous-time process model for the AHRS core.

    Responsibility:
        Specify the deterministic continuous-time dynamics and random-walk
        assumptions used to propagate the AHRS mean state and covariance.

    Purpose:
        Define the AHRS process dynamics and noise injection used by the EKF
        prediction step.

    Inputs/outputs:
        - Inputs: AhrsState, process noise intensities, Δt_ns.
        - Outputs: continuous-time Jacobians A, G and discrete F, Q.

    Dependencies:
        - Uses Quaternion for attitude kinematics.
        - Coupled with StateMapping and AhrsState definitions.

    Public API (to be implemented):
        - propagate_mean(state, dt_ns)
        - linearize(state)
        - noise_jacobian(state)
        - discretize(A, G, Q_c, dt_ns)

    Data contract:
        - state is an AhrsState instance.
        - A and G are N x N and N x q Jacobians, respectively.
        - Q_c is a continuous-time noise covariance with shape (q, q).
        - Canonical noise vector ordering (q = 39):
          [w_v, w_ω, w_bg, w_ba, w_A, w_BI, w_BM, w_g, w_m]
        - Ordering is canonical and must never change.
        - Block dimensions:
          w_v(3), w_ω(3), w_bg(3), w_ba(3), w_A(9),
          w_BI(6), w_BM(6), w_g(3), w_m(3)
        - G has shape (N, q) with N = StateMapping.dimension().

    Frames and units:
        - p_WB in meters, v_WB in meters per second.
        - omega_WB in rad/s.
        - Noise intensities follow Units and config definitions.

    Determinism and edge cases:
        - Dynamics are deterministic given the current state and Δt_ns.
        - IMU and mag measurements are not process inputs by design.
        - Random-walk mean states remain constant in propagation.
        - Δt_ns must be positive; Δt_ns <= 0 should be handled explicitly.
        - IMU/mag measurements are updates, not inputs to propagation.

    Equations:
        Mean propagation (zero-mean noise, no sampled injection):
            ṗ_WB = v_WB
            v̇_WB := 0

            q̇_WB = 0.5 * Ω(ω_WB) * q_WB
            ω̇_WB := 0

        Continuous-time noise model (covariance only, E[w_v] = 0, E[w_ω] = 0):
            v̇_WB = w_v
            ω̇_WB = w_ω

            ḃ_g = w_bg
            ḃ_a = w_ba
            vec(Ȧ_a) = w_A
            ξ̇_BI = w_BI
            ξ̇_BM = w_BM
            ġ_W = w_g
            ṁ_W = w_m

        First-order discretization:
            Define Δt_sec := Δt_ns * 1e-9 (deterministic scale; not used
            for keying).
            F ≈ I + AΔt_sec
            Q ≈ G Q_c Gᵀ Δt_sec

    Numerical stability notes:
        - Ensure F and Q are consistent with StateMapping.
        - Symmetrize Q if necessary after discretization.

    Suggested unit tests:
        - Random-walk states remain constant in mean propagation.
        - Discretization returns correctly shaped F and Q.
    """

    @staticmethod
    def propagate_mean(state: AhrsState, dt_ns: int) -> AhrsState:
        """Return propagated mean state for Δt_ns."""
        if not _is_int(dt_ns) or dt_ns <= 0:
            raise ValueError("dt_ns must be positive")
        dt_sec: float = dt_ns * 1e-9
        p_WB: List[float] = [
            state.p_WB[0] + state.v_WB[0] * dt_sec,
            state.p_WB[1] + state.v_WB[1] * dt_sec,
            state.p_WB[2] + state.v_WB[2] * dt_sec,
        ]
        v_WB: List[float] = _copy_vec(state.v_WB)
        q_WB: List[float] = Quaternion.integrate(state.q_WB, state.omega_WB, dt_sec)
        q_WB = Quaternion.normalize(q_WB)
        omega_WB: List[float] = _copy_vec(state.omega_WB)
        b_g: List[float] = _copy_vec(state.b_g)
        b_a: List[float] = _copy_vec(state.b_a)
        A_a: List[List[float]] = _copy_mat(state.A_a)
        T_BI: tuple[List[List[float]], List[float]] = _copy_transform(state.T_BI)
        T_BM: tuple[List[List[float]], List[float]] = _copy_transform(state.T_BM)
        g_W: List[float] = _copy_vec(state.g_W)
        m_W: List[float] = _copy_vec(state.m_W)
        return AhrsState(
            p_WB=p_WB,
            v_WB=v_WB,
            q_WB=q_WB,
            omega_WB=omega_WB,
            b_g=b_g,
            b_a=b_a,
            A_a=A_a,
            T_BI=T_BI,
            T_BM=T_BM,
            g_W=g_W,
            m_W=m_W,
        )

    @staticmethod
    def linearize(state: AhrsState) -> List[List[float]]:
        """Return the continuous-time Jacobian A."""
        _ = state
        size: int = StateMapping.dimension()
        A: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        p_slice: slice = StateMapping.slice_delta_p()
        v_slice: slice = StateMapping.slice_delta_v()
        i: int
        for i in range(3):
            A[p_slice.start + i][v_slice.start + i] = 1.0
        theta_slice: slice = StateMapping.slice_delta_theta()
        omega_slice: slice = StateMapping.slice_delta_omega()
        for i in range(3):
            A[theta_slice.start + i][omega_slice.start + i] = 1.0
        return A

    @staticmethod
    def noise_jacobian(state: AhrsState) -> List[List[float]]:
        """Return the continuous-time noise Jacobian G."""
        _ = state
        size: int = StateMapping.dimension()
        q_dim: int = 39
        G: List[List[float]] = [[0.0 for _ in range(q_dim)] for _ in range(size)]
        idx: int = 0
        v_slice: slice = StateMapping.slice_delta_v()
        i: int
        for i in range(3):
            G[v_slice.start + i][idx + i] = 1.0
        idx += 3
        omega_slice: slice = StateMapping.slice_delta_omega()
        for i in range(3):
            G[omega_slice.start + i][idx + i] = 1.0
        idx += 3
        b_g_slice: slice = StateMapping.slice_delta_b_g()
        for i in range(3):
            G[b_g_slice.start + i][idx + i] = 1.0
        idx += 3
        b_a_slice: slice = StateMapping.slice_delta_b_a()
        for i in range(3):
            G[b_a_slice.start + i][idx + i] = 1.0
        idx += 3
        A_slice: slice = StateMapping.slice_delta_A_a()
        for i in range(9):
            G[A_slice.start + i][idx + i] = 1.0
        idx += 9
        xi_bi_slice: slice = StateMapping.slice_delta_xi_BI()
        for i in range(6):
            G[xi_bi_slice.start + i][idx + i] = 1.0
        idx += 6
        xi_bm_slice: slice = StateMapping.slice_delta_xi_BM()
        for i in range(6):
            G[xi_bm_slice.start + i][idx + i] = 1.0
        idx += 6
        g_slice: slice = StateMapping.slice_delta_g_W()
        for i in range(3):
            G[g_slice.start + i][idx + i] = 1.0
        idx += 3
        m_slice: slice = StateMapping.slice_delta_m_W()
        for i in range(3):
            G[m_slice.start + i][idx + i] = 1.0
        return G

    @staticmethod
    def discretize(
        A: Sequence[Sequence[float]],
        G: Sequence[Sequence[float]],
        Q_c: Sequence[Sequence[float]],
        dt_ns: int,
    ) -> tuple[List[List[float]], List[List[float]]]:
        """Return discretized F and Q using first-order approximation."""
        if not _is_int(dt_ns) or dt_ns <= 0:
            raise ValueError("dt_ns must be positive")
        size_expected: int = StateMapping.dimension()
        if (
            len(A) != size_expected
            or any(len(row) != size_expected for row in A)
            or len(G) != size_expected
            or any(len(row) != 39 for row in G)
            or len(Q_c) != 39
            or any(len(row) != 39 for row in Q_c)
        ):
            raise ValueError("invalid matrix shape")
        dt_sec: float = dt_ns * 1e-9
        size: int = size_expected
        F: List[List[float]] = [[0.0 for _ in range(size)] for _ in range(size)]
        i: int
        j: int
        for i in range(size):
            for j in range(size):
                F[i][j] = A[i][j] * dt_sec
            F[i][i] += 1.0
        GQ: List[List[float]] = _matmul(G, Q_c)
        GQGt: List[List[float]] = _matmul(GQ, _transpose(G))
        Q: List[List[float]] = [[value * dt_sec for value in row] for row in GQGt]
        return (F, LinearAlgebra.symmetrize(Q))


def _is_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _copy_vec(vec: Sequence[float]) -> List[float]:
    """Return a copy of a vector."""
    return [float(value) for value in vec]


def _copy_mat(mat: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return a deep copy of a matrix."""
    return [[float(value) for value in row] for row in mat]


def _copy_transform(
    T: tuple[Sequence[Sequence[float]], Sequence[float]],
) -> tuple[List[List[float]], List[float]]:
    """Return a deep copy of a transform."""
    R, p = T
    return (_copy_mat(R), _copy_vec(p))


def _transpose(A: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return transpose of a matrix."""
    rows: int = len(A)
    cols: int = len(A[0])
    result: List[List[float]] = [[0.0 for _ in range(rows)] for _ in range(cols)]
    i: int
    j: int
    for i in range(rows):
        for j in range(cols):
            result[j][i] = A[i][j]
    return result


def _matmul(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Return A * B for matrices with compatible shapes."""
    rows: int = len(A)
    cols: int = len(B[0])
    inner: int = len(B)
    result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(rows)]
    i: int
    j: int
    k: int
    for i in range(rows):
        for j in range(cols):
            acc: float = 0.0
            for k in range(inner):
                acc += A[i][k] * B[k][j]
            result[i][j] = acc
    return result
