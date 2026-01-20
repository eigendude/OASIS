################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Linear-algebra helpers for AHRS covariance management.

Responsibility:
    Provide backend-agnostic helpers for symmetric matrices, SPD checks, and
    numerical conditioning needed by the filter and models.

Inputs/outputs:
    - Covariances are full matrices, never diagonalized.
    - Symmetrization is explicit: P <- 0.5 * (P + P^T).

Dependencies:
    - Used by covariance, update_step, and noise_adaptation modules.

Determinism:
    Deterministic behavior is required for SPD checks and clamping so that
    repeated runs produce identical decisions about accepting updates.
"""


class LinearAlgebra:
    """Linear-algebra helpers for full covariance handling.

    Purpose:
        Define the operations the AHRS needs for covariance symmetrization,
        SPD checking, and linear solves without enforcing a numpy backend.

    Public API (to be implemented):
        - symmetrize(P)
        - is_spd(P)
        - clamp_spd(P, min_eig, max_eig)
        - solve_spd(A, b)
        - cholesky(A)

    Data contract:
        - P, A are square matrices of shape (n, n).
        - b is a vector of shape (n,) or matrix of shape (n, m).

    Frames and units:
        - Covariance units follow the state units in Units.
        - No implicit unit conversions are performed.

    Determinism and edge cases:
        - is_spd must be deterministic for nearly singular matrices.
        - clamp_spd must preserve symmetry and clamp eigenvalues.

    Equations:
        Symmetrization:
            P_sym = 0.5 * (P + Pᵀ)

    Numerical stability notes:
        - clamp_spd should avoid negative eigenvalues caused by round-off.
        - solve_spd should prefer Cholesky for SPD matrices.

    Suggested unit tests:
        - symmetrize returns symmetric result for asymmetric input.
        - is_spd rejects matrices with non-positive eigenvalues.
        - clamp_spd keeps eigenvalues within [min_eig, max_eig].
    """

    pass
