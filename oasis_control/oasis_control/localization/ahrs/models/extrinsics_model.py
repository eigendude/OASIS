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
from typing import Tuple

from oasis_control.localization.ahrs.math_utils.se3 import Se3


Se3Transform = Tuple[List[List[float]], List[float]]


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

    @staticmethod
    def apply_delta(T_bx: Se3Transform, delta_xi: Sequence[float]) -> Se3Transform:
        """Return T <- Exp(δξ) * T using left perturbation."""
        _assert_transform("T_bx", T_bx)
        _assert_vector_length("delta_xi", delta_xi, 6)
        _assert_finite_vector("delta_xi", delta_xi)
        delta_T: Se3Transform = Se3.exp(delta_xi)
        return Se3.compose(delta_T, T_bx)

    @staticmethod
    def jacobian_wrt_delta() -> List[List[float]]:
        """Return the Jacobian of Exp(δξ) * T at δξ = 0."""
        # Left perturbations map directly to δξ at the identity
        return [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def adjoint(T_bx: Se3Transform) -> List[List[float]]:
        """Return the adjoint matrix Ad_T for T_bx."""
        _assert_transform("T_bx", T_bx)
        return Se3.adjoint(T_bx)


def _assert_vector_length(name: str, vec: Sequence[float], length: int) -> None:
    """Validate vector length."""
    if len(vec) != length:
        raise ValueError(f"{name} must have length {length}")


def _assert_matrix_shape(
    name: str,
    mat: Sequence[Sequence[float]],
    rows: int,
    cols: int,
) -> None:
    """Validate matrix shape."""
    if len(mat) != rows or any(len(row) != cols for row in mat):
        raise ValueError(f"{name} must be {rows}x{cols}")


def _assert_transform(name: str, T: Se3Transform) -> None:
    """Validate transform structure and finite entries."""
    if not isinstance(T, tuple) or len(T) != 2:
        raise ValueError(f"{name} must be a (R, p) tuple")
    R, p = T
    _assert_matrix_shape(f"{name}.R", R, 3, 3)
    _assert_vector_length(f"{name}.p", p, 3)
    _assert_finite_matrix(f"{name}.R", R)
    _assert_finite_vector(f"{name}.p", p)


def _assert_finite_vector(name: str, vec: Sequence[float]) -> None:
    """Validate vector entries are finite."""
    value: float
    for value in vec:
        if not math.isfinite(value):
            raise ValueError(f"{name} contains non-finite value")


def _assert_finite_matrix(name: str, mat: Sequence[Sequence[float]]) -> None:
    """Validate matrix entries are finite."""
    row: Sequence[float]
    for row in mat:
        _assert_finite_vector(name, row)
