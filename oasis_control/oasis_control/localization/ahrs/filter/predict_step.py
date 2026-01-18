################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class PredictStep:
    """Prediction step for the AHRS EKF.

    Responsibility:
        Document the propagation of the mean state and covariance using the
        process model and first-order discretization.

    Purpose:
        Propagate the mean state and covariance forward in time using the
        continuous-time process model and first-order discretization.

    Inputs/outputs:
        - Inputs: AhrsState, AhrsCovariance, time step Δt_ns.
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
            F ≈ I + AΔt_ns
            Q ≈ G Q_c Gᵀ Δt_ns

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

    pass
