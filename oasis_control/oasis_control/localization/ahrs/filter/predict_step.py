################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Prediction step for the AHRS EKF.

Responsibility:
    Document the propagation of the mean state and covariance using the
    process model and first-order discretization.

Inputs/outputs:
    - Inputs: AhrsState, AhrsCovariance, time step Δt.
    - Outputs: propagated state and covariance.

Dependencies:
    - Depends on ProcessModel, StateMapping, and LinearAlgebra.

Determinism:
    Prediction uses deterministic ordering and must be applied before any
    measurement updates at the same timestamp.
"""


class PredictStep:
    """EKF prediction step for the AHRS state and covariance.

    Purpose:
        Propagate the mean state and covariance forward in time using the
        continuous-time process model and first-order discretization.

    Public API (to be implemented):
        - propagate(state, covariance, dt)
        - compute_jacobians(state)
        - discretize(A, G, Q_c, dt)

    Data contract:
        - state is an AhrsState instance.
        - covariance P is N x N with N from StateMapping.
        - dt is a positive float in seconds.

    Frames and units:
        - dt in seconds.
        - State units follow Units.

    Determinism and edge cases:
        - IMU and magnetometer samples are not process inputs.
        - Random-walk mean states are held constant during propagation.
        - dt <= 0 should result in a no-op or a rejected step.

    Equations:
        First-order discretization:
            F ≈ I + AΔt
            Q ≈ G Q_c Gᵀ Δt

        Mean state propagation (random walks hold mean constant):
            ṗ = v
            v̇ = w_v
            q̇ = 0.5 * Ω(ω) * q
            ω̇ = w_ω
            b_g, b_a, A_a, T_BI, T_BM, g_W, m_W are random walks

    Numerical stability notes:
        - Symmetrize covariance after propagation.
        - Ensure Q remains SPD.

    Suggested unit tests:
        - dt=0 leaves state unchanged.
        - F and Q shapes are consistent with StateMapping.
    """

    pass
