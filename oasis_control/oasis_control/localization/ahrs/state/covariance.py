################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class AhrsCovariance:
    """Covariance container and operations for the AHRS EKF.

    Responsibility:
        Store and document the full error-state covariance matrix used by the
        EKF, including symmetrization and mapping rules.

    Purpose:
        Maintain the error-state covariance matrix with explicit symmetrization
        and mapping rules between coordinate frames.

    Inputs/outputs:
        - P is an N x N full covariance matrix.
        - Jacobian-based output covariance uses Σ_y = J P J^T.

    Dependencies:
        - Uses StateMapping for ordering and LinearAlgebra for symmetrization.

    Public API (to be implemented):
        - symmetrize()
        - as_matrix()
        - from_matrix(P)
        - map_with_jacobian(J)
        - apply_adjoint(Ad_T)

    Data contract:
        - P is an N x N matrix with N defined by StateMapping.
        - All covariance blocks are full and symmetric.

    Frames and units:
        - Covariance units are derived from Units for each state element.
        - SE(3) covariance mapping uses the adjoint of the transform.

    Determinism and edge cases:
        - Symmetrization must be applied deterministically after any update.
        - Always apply P <- 0.5 * (P + P^T) after updates.
        - Mapping with Jacobians must preserve symmetry.

    Equations:
        Output covariance for y = f(x):
            Σ_y = J P J^T

        SE(3) adjoint mapping:
            Σ_A = Ad_T * Σ_B * Ad_T^T

    Numerical stability notes:
        - Symmetrize after each update to mitigate round-off.
        - Avoid implicit diagonalization.

    Suggested unit tests:
        - symmetrize yields symmetric matrix.
        - map_with_jacobian matches manual multiplication.
    """

    pass
