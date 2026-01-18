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

import math
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
        - update_mag_covariance(R_m, nu, S_hat, alpha, min_eig, max_eig)
        - clamp_spd(P, min_eig, max_eig)

    Data contract:
        - R_m is a 3x3 SPD matrix.
        - nu is a length-3 residual in {M}.
        - S_hat is innovation covariance without R_m.
        - min_eig and max_eig are scalar eigenvalue bounds.

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
            R_m <- clamp_SPD((1-α) R_m + α (ν νᵀ - S_hat), min_eig, max_eig)

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
        min_eig: float,
        max_eig: float,
    ) -> List[List[float]]:
        """Return adapted magnetometer covariance."""
        _assert_matrix_shape("R_m", R_m, 3)
        _assert_matrix_shape("S_hat", S_hat, 3)
        if len(nu) != 3:
            raise ValueError("nu must have length 3")
        if not math.isfinite(alpha) or alpha < 0.0 or alpha > 1.0:
            raise ValueError("alpha must be in [0, 1]")
        if (
            not math.isfinite(min_eig)
            or not math.isfinite(max_eig)
            or min_eig <= 0.0
            or max_eig < min_eig
        ):
            raise ValueError("eigenvalue bounds invalid")
        nu_outer: List[List[float]] = [
            [nu[0] * nu[0], nu[0] * nu[1], nu[0] * nu[2]],
            [nu[1] * nu[0], nu[1] * nu[1], nu[1] * nu[2]],
            [nu[2] * nu[0], nu[2] * nu[1], nu[2] * nu[2]],
        ]
        weighted_R: List[List[float]] = _scale_mat(R_m, 1.0 - alpha)
        innovation_term: List[List[float]] = _sub_mat(nu_outer, S_hat)
        innovation_scaled: List[List[float]] = _scale_mat(innovation_term, alpha)
        R_new: List[List[float]] = _add_mat(weighted_R, innovation_scaled)
        R_new = LinearAlgebra.symmetrize(R_new)
        return NoiseAdaptation.clamp_spd(R_new, min_eig, max_eig)

    @staticmethod
    def clamp_spd(
        P: Sequence[Sequence[float]],
        min_eig: float,
        max_eig: float,
    ) -> List[List[float]]:
        """Clamp eigenvalues of P to [min_eig, max_eig]."""
        return LinearAlgebra.clamp_spd(P, min_eig, max_eig)


def _assert_matrix_shape(name: str, mat: Sequence[Sequence[float]], size: int) -> None:
    """Validate matrix shape."""
    if len(mat) != size or any(len(row) != size for row in mat):
        raise ValueError(f"{name} must be {size}x{size}")


def _add_mat(
    A: Sequence[Sequence[float]], B: Sequence[Sequence[float]]
) -> List[List[float]]:
    """Return A + B for 3x3 matrices."""
    return [
        [A[0][0] + B[0][0], A[0][1] + B[0][1], A[0][2] + B[0][2]],
        [A[1][0] + B[1][0], A[1][1] + B[1][1], A[1][2] + B[1][2]],
        [A[2][0] + B[2][0], A[2][1] + B[2][1], A[2][2] + B[2][2]],
    ]


def _sub_mat(
    A: Sequence[Sequence[float]], B: Sequence[Sequence[float]]
) -> List[List[float]]:
    """Return A - B for 3x3 matrices."""
    return [
        [A[0][0] - B[0][0], A[0][1] - B[0][1], A[0][2] - B[0][2]],
        [A[1][0] - B[1][0], A[1][1] - B[1][1], A[1][2] - B[1][2]],
        [A[2][0] - B[2][0], A[2][1] - B[2][1], A[2][2] - B[2][2]],
    ]


def _scale_mat(A: Sequence[Sequence[float]], scale: float) -> List[List[float]]:
    """Return scale * A for 3x3 matrices."""
    return [
        [A[0][0] * scale, A[0][1] * scale, A[0][2] * scale],
        [A[1][0] * scale, A[1][1] * scale, A[1][2] * scale],
        [A[2][0] * scale, A[2][1] * scale, A[2][2] * scale],
    ]
