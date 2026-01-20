################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""AHRS se3 definitions."""


class Se3:
    """
    SE(3) utilities for extrinsics and pose perturbations.

    Each transform T_AB = (R_AB, p_AB) maps vectors via:

        p_A = R_AB * p_B + p_AB

    Perturbations are represented in the tangent space:

        δξ = [δρ; δθ]

    The perturbed rotation uses the exponential map:

        R ≈ Exp(δθ) * R_hat

    The adjoint operator transforms covariance blocks:

        Ad_T = [[R, [p]× R], [0, R]]
        Σ_A = Ad_T * Σ_B * Ad_Tᵀ
    """

    pass
