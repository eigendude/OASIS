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

from typing import List, Sequence

from oasis_control.localization.ahrs.math_utils.linalg import LinearAlgebra


class Statistics:
    """Statistical utilities for innovation metrics and gating.

    Responsibility:
        Document innovation covariance, Mahalanobis distance, and gating
        rules used during measurement updates.

    Purpose:
        Provide the equations and helper operations for innovation covariance
        and Mahalanobis distance used in gating decisions.

    Inputs/outputs:
        - Innovation residual ν is length m.
        - Innovation covariance S is m x m.
        - Mahalanobis distance is a scalar νᵀ S⁻¹ ν.

    Dependencies:
        - Used by update_step and noise_adaptation.

    Public API (to be implemented):
        - innovation_covariance(H, P, R)
        - mahalanobis_squared(nu, S)
        - gating_passes(nu, S, threshold)

    Data contract:
        - H is an m x N measurement Jacobian.
        - P is an N x N state covariance.
        - R is an m x m measurement covariance.
        - nu is a length-m residual vector.

    Frames and units:
        - Residual units follow the measurement units.
        - S units are squared measurement units.

    Determinism and edge cases:
        - Gating decisions are deterministic for identical S and ν.
        - If S is not SPD or inversion fails, gating must reject the update.
        - Thresholds are provided explicitly by configuration.

    Equations:
        Innovation covariance:
            S = H P Hᵀ + R

        Mahalanobis distance:
            d² = νᵀ S⁻¹ ν

    Numerical stability notes:
        - Use SPD solvers to avoid explicit inversion where possible.
        - Symmetrize S before SPD checks if small asymmetry is present.

    Suggested unit tests:
        - d² is zero when ν is zero.
        - gating_passes rejects when d² exceeds the threshold.
    """

    @staticmethod
    def innovation_covariance(
        H: Sequence[Sequence[float]],
        P: Sequence[Sequence[float]],
        R: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        """Return innovation covariance S = H P H^T + R."""
        if not H:
            raise ValueError("H must be non-empty")
        m: int = len(H)
        n: int = len(H[0])
        if any(len(row) != n for row in H):
            raise ValueError("H must have consistent row lengths")
        if len(P) != n or any(len(row) != n for row in P):
            raise ValueError("P must be square with shape (n, n)")
        if len(R) != m or any(len(row) != m for row in R):
            raise ValueError("R must be square with shape (m, m)")
        HP: List[List[float]] = [[0.0 for _ in range(n)] for _ in range(m)]
        for i in range(m):
            for j in range(n):
                acc: float = 0.0
                for k in range(n):
                    acc += H[i][k] * P[k][j]
                HP[i][j] = acc
        S: List[List[float]] = [[0.0 for _ in range(m)] for _ in range(m)]
        for i in range(m):
            for j in range(m):
                acc = 0.0
                for k in range(n):
                    acc += HP[i][k] * H[j][k]
                S[i][j] = acc + R[i][j]
        return S

    @staticmethod
    def mahalanobis_squared(nu: Sequence[float], S: Sequence[Sequence[float]]) -> float:
        """Return the squared Mahalanobis distance using SPD solve."""
        if len(S) != len(nu):
            raise ValueError("S dimension must match nu")
        x = LinearAlgebra.solve_spd(S, nu)
        x_vec: Sequence[float] = x  # type: ignore[assignment]
        d2: float = 0.0
        for i in range(len(nu)):
            d2 += nu[i] * x_vec[i]
        return d2

    @staticmethod
    def gating_passes(
        nu: Sequence[float],
        S: Sequence[Sequence[float]],
        threshold: float,
    ) -> bool:
        """Return True if the Mahalanobis distance is within threshold."""
        try:
            d2: float = Statistics.mahalanobis_squared(nu, S)
        except ValueError:
            return False
        return d2 <= threshold
