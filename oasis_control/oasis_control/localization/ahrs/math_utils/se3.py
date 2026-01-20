################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

class Se3:
    """SE(3) utilities for AHRS extrinsics and perturbations.

    Responsibility:
        Define rigid-body transforms and adjoint mappings for extrinsics used
        in the AHRS state and error-state models.

    Purpose:
        Provide the rigid-body transform conventions and operators needed to
        express IMU/magnetometer extrinsics in a ROS-agnostic core.

    Inputs/outputs:
        - Transform T_AB = (R_AB, p_AB) maps vectors from {B} to {A}.
        - Rotation matrices are 3x3, translations are 3x1.
        - Tangent vectors δξ are 6x1 with [δρ; δθ].

    Dependencies:
        - Used by extrinsics_model, state_mapping, and covariance transforms.
        - Works with Quaternion and LinearAlgebra utilities for consistency.

    Public API (to be implemented):
        - compose(T_ab, T_bc)
        - inverse(T_ab)
        - apply(T_ab, p_b)
        - to_matrix(T_ab)
        - from_matrix(T_ab_matrix)
        - exp(delta_xi)
        - log(T_ab)
        - hat(delta_xi)
        - vee(xi_hat)
        - adjoint(T_ab)

    Data contract:
        - R is a 3x3 rotation matrix.
        - p is a 3x1 translation vector in meters.
        - delta_xi is a 6x1 vector [delta_rho; delta_theta].

    Frames and units:
        - T_AB maps p_B to p_A via p_A = R_AB * p_B + p_AB.
        - delta_rho in meters, delta_theta in radians.

    Determinism and edge cases:
        - Transform composition and adjoint calculations are deterministic
          and numerically stable for small perturbations.
        - exp/log must be consistent for small-angle perturbations.
        - adjoint must be defined for any valid T_AB.

    Equations:
        Perturbation composition:
            T = Exp(δξ) * T_hat

        Adjoint for covariance mapping:
            Ad_T = [[R, [p]× R], [0, R]]
            Σ_A = Ad_T * Σ_B * Ad_Tᵀ

    Numerical stability notes:
        - Use series expansions for small rotation angles.
        - Avoid catastrophic cancellation when ||delta_theta|| is small.

    Suggested unit tests:
        - compose(inverse(T), T) yields identity.
        - exp(log(T)) returns T within tolerance.
        - adjoint matches finite-difference covariance mapping.
    """

    pass
