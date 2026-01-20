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
        """Return the deterministic mean propagation of the state."""
        _assert_dt_ns(dt_ns)
        dt_sec: float = dt_ns * 1e-9
        p_WB: List[float] = [
            state.p_WB[0] + state.v_WB[0] * dt_sec,
            state.p_WB[1] + state.v_WB[1] * dt_sec,
            state.p_WB[2] + state.v_WB[2] * dt_sec,
        ]
        q_WB: List[float] = Quaternion.integrate(state.q_WB, state.omega_WB, dt_sec)
        q_WB = Quaternion.normalize(q_WB)
        return AhrsState(
            p_WB=p_WB,
            v_WB=list(state.v_WB),
            q_WB=q_WB,
            omega_WB=list(state.omega_WB),
            b_g=list(state.b_g),
            b_a=list(state.b_a),
            A_a=[list(row) for row in state.A_a],
            T_BI=(
                [list(row) for row in state.T_BI[0]],
                list(state.T_BI[1]),
            ),
            T_BM=(
                [list(row) for row in state.T_BM[0]],
                list(state.T_BM[1]),
            ),
            g_W=list(state.g_W),
            m_W=list(state.m_W),
        )

    @staticmethod
    def linearize(state: AhrsState) -> List[List[float]]:
        """Return the continuous-time Jacobian A for the mean dynamics."""
        _ = state
        size: int = StateMapping.dimension()
        A: List[List[float]] = _zeros_matrix(size, size)
        delta_p: slice = StateMapping.slice_delta_p()
        delta_v: slice = StateMapping.slice_delta_v()
        delta_theta: slice = StateMapping.slice_delta_theta()
        delta_omega: slice = StateMapping.slice_delta_omega()
        for i in range(3):
            A[delta_p.start + i][delta_v.start + i] = 1.0
            A[delta_theta.start + i][delta_omega.start + i] = 1.0
        return A

    @staticmethod
    def noise_jacobian(state: AhrsState) -> List[List[float]]:
        """Return the noise Jacobian G for the canonical noise ordering."""
        _ = state
        n: int = StateMapping.dimension()
        q: int = 39
        G: List[List[float]] = _zeros_matrix(n, q)

        noise_index: int = 0
        _insert_identity(G, StateMapping.slice_delta_v(), noise_index)
        noise_index += 3
        _insert_identity(G, StateMapping.slice_delta_omega(), noise_index)
        noise_index += 3
        _insert_identity(G, StateMapping.slice_delta_b_g(), noise_index)
        noise_index += 3
        _insert_identity(G, StateMapping.slice_delta_b_a(), noise_index)
        noise_index += 3
        _insert_identity(G, StateMapping.slice_delta_A_a(), noise_index)
        noise_index += 9
        _insert_identity(G, StateMapping.slice_delta_xi_BI(), noise_index)
        noise_index += 6
        _insert_identity(G, StateMapping.slice_delta_xi_BM(), noise_index)
        noise_index += 6
        _insert_identity(G, StateMapping.slice_delta_g_W(), noise_index)
        noise_index += 3
        _insert_identity(G, StateMapping.slice_delta_m_W(), noise_index)
        return G

    @staticmethod
    def discretize(
        A: Sequence[Sequence[float]],
        G: Sequence[Sequence[float]],
        Q_c: Sequence[Sequence[float]],
        dt_ns: int,
    ) -> tuple[list[list[float]], list[list[float]]]:
        """Return first-order discretized (F, Q)."""
        _assert_dt_ns(dt_ns)
        dt_sec: float = dt_ns * 1e-9
        n: int = len(A)
        if any(len(row) != n for row in A):
            raise ValueError("A must be square")
        if len(G) != n:
            raise ValueError("G must have same row count as A")
        q: int = len(Q_c)
        if any(len(row) != q for row in Q_c):
            raise ValueError("Q_c must be square")
        if any(len(row) != q for row in G):
            raise ValueError("G must have q columns")

        F: List[List[float]] = _identity(n)
        for i in range(n):
            for j in range(n):
                F[i][j] += A[i][j] * dt_sec

        GQ: List[List[float]] = _zeros_matrix(n, q)
        for i in range(n):
            for j in range(q):
                acc: float = 0.0
                for k in range(q):
                    acc += G[i][k] * Q_c[k][j]
                GQ[i][j] = acc
        Q: List[List[float]] = _zeros_matrix(n, n)
        for i in range(n):
            for j in range(n):
                acc = 0.0
                for k in range(q):
                    acc += GQ[i][k] * G[j][k]
                Q[i][j] = acc * dt_sec
        Q = LinearAlgebra.symmetrize(Q)
        return F, Q


def _assert_dt_ns(dt_ns: int) -> None:
    if not isinstance(dt_ns, int) or isinstance(dt_ns, bool):
        raise ValueError("dt_ns must be positive int nanoseconds")
    if dt_ns <= 0:
        raise ValueError("dt_ns must be positive int nanoseconds")


def _zeros_matrix(rows: int, cols: int) -> List[List[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def _identity(n: int) -> List[List[float]]:
    eye: List[List[float]] = _zeros_matrix(n, n)
    for i in range(n):
        eye[i][i] = 1.0
    return eye


def _insert_identity(J: List[List[float]], slc: slice, col_start: int) -> None:
    if slc.start is None or slc.stop is None:
        raise ValueError("slice must be bounded")
    length: int = slc.stop - slc.start
    for i in range(length):
        J[slc.start + i][col_start + i] = 1.0
