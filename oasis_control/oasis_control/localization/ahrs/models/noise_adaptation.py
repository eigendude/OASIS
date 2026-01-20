################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

from __future__ import annotations

from typing import List
from typing import Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


class NoiseAdaptation:
    """Adaptive measurement noise handling for magnetometer updates.

    Responsibility:
        Specify the adaptive covariance update for magnetometer measurements
        and document the SPD clamping requirements.

    Purpose:
        Provide the R_m adaptation rule and SPD clamping behavior used during
        magnetometer updates.

    Inputs/outputs:
        - Inputs: residual nu, predicted innovation covariance S_hat.
        - Outputs: adapted R_m covariance.

    Dependencies:
        - Uses LinearAlgebra for SPD checks and clamping.
        - Consumed by MagModel and UpdateStep.

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
        - Adaptive updates are deterministic for identical inputs and clamp
          limits.
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

    @staticmethod
    def update_mag_covariance(
        R_m: Sequence[Sequence[float]],
        nu: Sequence[float],
        S_hat: Sequence[Sequence[float]],
        alpha: float,
        R_min: float,
        R_max: float,
    ) -> List[List[float]]:
        """Return the adapted magnetometer covariance."""
        if len(nu) != 3:
            raise ValueError("nu must have length 3")
        if len(R_m) != 3 or any(len(row) != 3 for row in R_m):
            raise ValueError("R_m must be 3x3")
        if len(S_hat) != 3 or any(len(row) != 3 for row in S_hat):
            raise ValueError("S_hat must be 3x3")
        nu_outer: List[List[float]] = [
            [nu[0] * nu[0], nu[0] * nu[1], nu[0] * nu[2]],
            [nu[1] * nu[0], nu[1] * nu[1], nu[1] * nu[2]],
            [nu[2] * nu[0], nu[2] * nu[1], nu[2] * nu[2]],
        ]
        R_new: List[List[float]] = [[0.0 for _ in range(3)] for _ in range(3)]
        for i in range(3):
            for j in range(3):
                R_new[i][j] = (1.0 - alpha) * R_m[i][j] + alpha * (
                    nu_outer[i][j] - S_hat[i][j]
                )
        R_new = LinearAlgebra.symmetrize(R_new)
        return NoiseAdaptation.clamp_spd(R_new, R_min, R_max)

    @staticmethod
    def clamp_spd(
        P: Sequence[Sequence[float]],
        min_eig: float,
        max_eig: float,
    ) -> List[List[float]]:
        """Clamp SPD matrix eigenvalues into [min_eig, max_eig]."""
        return LinearAlgebra.clamp_spd(P, min_eig, max_eig)
