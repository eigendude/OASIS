################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Measurement update step for the AHRS EKF.

Responsibility:
    Document the generic EKF update equations, Joseph-form covariance update,
    and rejection policies for invalid innovation covariance.

Inputs/outputs:
    - Inputs: P (N x N), H (m x N), R (m x m), nu (m,), z, z_hat.
    - Outputs: updated state correction and covariance, plus UpdateReport.

Dependencies:
    - Uses Statistics for innovation metrics and LinearAlgebra for SPD checks.

Determinism:
    Updates must be deterministic and applied in the prescribed order.
"""


class UpdateStep:
    """Generic EKF measurement update for AHRS.

    Purpose:
        Apply a measurement update using the EKF equations with a Joseph-form
        covariance update and explicit rejection rules.

    Public API (to be implemented):
        - compute_innovation(H, P, R, nu)
        - compute_kalman_gain(P, H, S)
        - apply_update(state, covariance, K, nu)
        - update_report(...)

    Data contract:
        - P: N x N covariance matrix.
        - H: m x N measurement Jacobian.
        - R: m x m measurement covariance.
        - nu: length-m innovation residual.
        - z and z_hat: length-m measurement and prediction.

    Frames and units:
        - nu shares the measurement frame ({I} for IMU, {M} for mag).
        - Covariance units align with Units.

    Determinism and edge cases:
        - If S is not SPD or solving fails, reject the update and record it
          in UpdateReport.
        - On rejection, state and covariance remain unchanged (no partial
          update).
        - Symmetrize P after applying the Joseph-form update.

    Equations:
        Innovation and gain:
            S = H P Hᵀ + R
            K = P Hᵀ S⁻¹
            δx = K ν

        Joseph-form covariance update:
            P <- (I - K H) P (I - K H)ᵀ + K R Kᵀ

    Numerical stability notes:
        - Use SPD solvers for S to avoid explicit inversion.
        - Symmetrize P after the update.

    Suggested unit tests:
        - Reject updates when S is not SPD.
        - Joseph-form update preserves symmetry.
    """

    pass
