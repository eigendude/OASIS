################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Adaptive measurement noise handling for magnetometer updates.

Responsibility:
    Specify the adaptive covariance update for magnetometer measurements and
    document the SPD clamping requirements.

Inputs/outputs:
    - Inputs: residual nu, predicted innovation covariance S_hat.
    - Outputs: adapted R_m covariance.

Dependencies:
    - Uses LinearAlgebra for SPD checks and clamping.
    - Consumed by MagModel and UpdateStep.

Determinism:
    Adaptive updates must be deterministic for identical inputs and clamp
    limits.
"""


class NoiseAdaptation:
    """Adaptive magnetometer covariance update logic.

    Purpose:
        Provide the R_m adaptation rule and SPD clamping behavior used during
        magnetometer updates.

    Public API (to be implemented):
        - update_mag_covariance(R_m, nu, S_hat, alpha, R_min, R_max)
        - clamp_spd(P, min_eig, max_eig)

    Data contract:
        - R_m, R_min, R_max are 3x3 SPD matrices.
        - nu is a length-3 residual in {M}.
        - S_hat is innovation covariance without R_m.

    Frames and units:
        - nu is in {M}, units tesla.
        - R_m is in tesla^2.

    Determinism and edge cases:
        - clamp_SPD preserves symmetry and clamps eigenvalues.
        - If nu nu^T - S_hat is not SPD, clamping still enforces SPD bounds.

    Equations:
        Adaptive update:
            R_m <- clamp_SPD((1-α) R_m + α (ν νᵀ - S_hat), R_min, R_max)

    Numerical stability notes:
        - Symmetrize before clamping.
        - Avoid negative eigenvalues after subtraction.

    Suggested unit tests:
        - Eigenvalues are within bounds after clamping.
        - Symmetry is preserved after update.
    """

    pass
