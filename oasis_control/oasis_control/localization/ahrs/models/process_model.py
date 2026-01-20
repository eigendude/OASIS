################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Continuous-time process model for the AHRS core.

Responsibility:
    Specify the deterministic continuous-time dynamics and random-walk
    assumptions used to propagate the AHRS mean state and covariance.

Inputs/outputs:
    - Inputs: AhrsState, process noise intensities, dt.
    - Outputs: continuous-time Jacobians A, G and discrete F, Q.

Dependencies:
    - Uses Quaternion for attitude kinematics.
    - Coupled with StateMapping and AhrsState definitions.

Determinism:
    Dynamics are deterministic given the current state and dt. IMU and mag
    measurements are not process inputs by design.
"""


class ProcessModel:
    """Continuous-time process model for smooth motion and drift.

    Purpose:
        Define the AHRS process dynamics and noise injection used by the EKF
        prediction step.

    Public API (to be implemented):
        - propagate_mean(state, dt)
        - linearize(state)
        - noise_jacobian(state)
        - discretize(A, G, Q_c, dt)

    Data contract:
        - state is an AhrsState instance.
        - A and G are N x N and N x q Jacobians, respectively.
        - Q_c is a continuous-time noise covariance.

    Frames and units:
        - p_WB in meters, v_WB in meters per second.
        - omega_WB in rad/s.
        - Noise intensities follow Units and config definitions.

    Determinism and edge cases:
        - Random-walk mean states remain constant in propagation.
        - dt must be positive; dt <= 0 should be handled explicitly.
        - IMU/mag measurements are updates, not inputs to propagation.

    Equations:
        Navigation kinematics:
            ṗ_WB = v_WB
            v̇_WB = w_v

        Attitude kinematics:
            q̇_WB = 0.5 * Ω(ω_WB) * q_WB
            ω̇_WB = w_ω

        Random walks:
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
