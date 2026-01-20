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
        h: Sequence[Sequence[float]],
        p: Sequence[Sequence[float]],
        r: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        Statistics._validate_matrix(h, "innovation_covariance")
        Statistics._validate_matrix(p, "innovation_covariance")
        Statistics._validate_matrix(r, "innovation_covariance")
        hp: List[List[float]] = Statistics._matmul(h, p)
        hph_t: List[List[float]] = Statistics._matmul(hp, Statistics._transpose(h))
        return Statistics._matadd(hph_t, r)

    @staticmethod
    def mahalanobis_squared(nu: Sequence[float], s: Sequence[Sequence[float]]) -> float:
        Statistics._validate_vector(nu, "mahalanobis_squared")
        Statistics._validate_matrix(s, "mahalanobis_squared")
        x: List[float] = LinearAlgebra.solve_spd(s, nu)  # type: ignore[assignment]
        return sum(nu[i] * x[i] for i in range(len(nu)))

    @staticmethod
    def gating_passes(
        nu: Sequence[float],
        s: Sequence[Sequence[float]],
        threshold: float,
    ) -> bool:
        try:
            d2: float = Statistics.mahalanobis_squared(nu, s)
        except ValueError:
            return False
        return d2 <= threshold

    @staticmethod
    def _matmul(
        left: Sequence[Sequence[float]],
        right: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        rows: int = len(left)
        cols: int = len(right[0])
        inner: int = len(right)
        result: List[List[float]] = [[0.0 for _ in range(cols)] for _ in range(rows)]
        for i in range(rows):
            for k in range(inner):
                for j in range(cols):
                    result[i][j] += left[i][k] * right[k][j]
        return result

    @staticmethod
    def _transpose(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
        return [list(row) for row in zip(*matrix)]

    @staticmethod
    def _matadd(
        left: Sequence[Sequence[float]],
        right: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        if len(left) != len(right):
            raise ValueError("innovation_covariance dimension mismatch")
        return [
            [left[i][j] + right[i][j] for j in range(len(left[0]))]
            for i in range(len(left))
        ]

    @staticmethod
    def _validate_vector(vec: Sequence[float], name: str) -> None:
        if not vec:
            raise ValueError(f"{name} expects a non-empty vector")

    @staticmethod
    def _validate_matrix(matrix: Sequence[Sequence[float]], name: str) -> None:
        if not matrix:
            raise ValueError(f"{name} expects a non-empty matrix")
        cols: int = len(matrix[0])
        for row in matrix:
            if len(row) != cols:
                raise ValueError(f"{name} expects rectangular matrix")
