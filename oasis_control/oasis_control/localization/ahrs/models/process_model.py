################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

class ProcessModel:
    """Continuous-time process model for the AHRS core.

    Responsibility:
        Specify the deterministic continuous-time dynamics and random-walk
        assumptions used to propagate the AHRS mean state and covariance.

    Purpose:
        Define the AHRS process dynamics and noise injection used by the EKF
        prediction step.

    Inputs/outputs:
        - Inputs: AhrsState, process noise intensities, dt.
        - Outputs: continuous-time Jacobians A, G and discrete F, Q.

    Dependencies:
        - Uses Quaternion for attitude kinematics.
        - Coupled with StateMapping and AhrsState definitions.

    Public API (to be implemented):
        - propagate_mean(state, dt)
        - linearize(state)
        - noise_jacobian(state)
        - discretize(A, G, Q_c, dt)

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
        - Dynamics are deterministic given the current state and dt.
        - IMU and mag measurements are not process inputs by design.
        - Random-walk mean states remain constant in propagation.
        - dt must be positive; dt <= 0 should be handled explicitly.
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
            F ≈ I + AΔt
            Q ≈ G Q_c Gᵀ Δt

    Numerical stability notes:
        - Ensure F and Q are consistent with StateMapping.
        - Symmetrize Q if necessary after discretization.

    Suggested unit tests:
        - Random-walk states remain constant in mean propagation.
        - Discretization returns correctly shaped F and Q.
    """

    pass
