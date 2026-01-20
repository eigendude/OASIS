################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS process model definitions."""

class ProcessModel:
    """
    Continuous-time process model for smooth motion and drift.

    Navigation kinematics:

        ṗ_WB = v_WB
        v̇_WB = w_v

    Attitude kinematics:

        q̇_WB = 0.5 * Ω(ω_WB) * q_WB
        ω̇_WB = w_ω

    Systematic drift terms are random walks:

        ḃ_g = w_bg
        ḃ_a = w_ba
        vec(Ȧ_a) = w_A
        δξ̇_BI = w_BI
        δξ̇_BM = w_BM
        ġ_W = w_g
        ṁ_W = w_m

    Discretization over Δt uses first-order approximations:

        F ≈ I + AΔt
        Q ≈ G Q_c Gᵀ Δt
    """
    pass
