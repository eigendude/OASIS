################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class ExtrinsicsModel:
    """Extrinsics modeling utilities for the AHRS core.

    Responsibility:
        Document how IMU and magnetometer extrinsics are represented and
        updated as SE(3) transforms with random-walk dynamics.

    Purpose:
        Provide conventions for IMU and magnetometer extrinsics so that
        updates remain consistent between state and covariance.

    Inputs/outputs:
        - Inputs: AhrsState extrinsics T_BI and T_BM.
        - Outputs: SE(3) perturbations and adjoint mappings.

    Dependencies:
        - Uses Se3 for Exp/Log and adjoint calculations.
        - Used by process and measurement models.

    Public API (to be implemented):
        - apply_delta(T_bx, delta_xi)
        - jacobian_wrt_delta()
        - adjoint(T_bx)

    Data contract:
        - T_BI and T_BM are SE(3) transforms from sensor to body.
        - delta_xi is a 6x1 perturbation vector [delta_rho; delta_theta].

    Frames and units:
        - T_BI maps vectors from {I} to {B}.
        - T_BM maps vectors from {M} to {B}.
        - delta_rho in meters, delta_theta in radians.

    Determinism and edge cases:
        - Extrinsics updates are deterministic given the error-state
          perturbation.
        - Perturbations apply on the left: T <- Exp(delta_xi) * T.
        - Small-angle assumptions must remain valid.

    Equations:
        - SE(3) perturbation uses Exp(delta_xi).
        - Adjoint mapping for covariance: Σ_A = Ad_T Σ_B Ad_T^T.

    Numerical stability notes:
        - Use small-angle series expansion for Exp when needed.

    Suggested unit tests:
        - apply_delta with zero perturbation returns original T.
        - Adjoint mapping preserves symmetry.
    """

    pass
