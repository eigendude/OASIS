################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Small 3D linear algebra helpers."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .units import PhysicalConstants
from .units import assert_finite


class Vec3:
    """Vector utilities for 3D vectors."""

    @staticmethod
    def normalize(
        v: NDArray[np.float64],
        eps: float = PhysicalConstants.EPS,
    ) -> NDArray[np.float64]:
        """Normalize a 3-vector."""
        vec: NDArray[np.float64] = np.asarray(v, dtype=float)
        if vec.shape != (3,):
            raise ValueError("v must be shape (3,)")
        assert_finite(vec, "v")
        norm: float = float(np.linalg.norm(vec))
        if norm < eps:
            raise ValueError("v has near-zero norm")
        return vec / norm

    @staticmethod
    def angle_between(
        u: NDArray[np.float64],
        v: NDArray[np.float64],
        eps: float = PhysicalConstants.EPS,
    ) -> float:
        """Return the angle between two 3-vectors in radians."""
        u_vec: NDArray[np.float64] = np.asarray(u, dtype=float)
        v_vec: NDArray[np.float64] = np.asarray(v, dtype=float)
        if u_vec.shape != (3,) or v_vec.shape != (3,):
            raise ValueError("u and v must be shape (3,)")
        assert_finite(u_vec, "u")
        assert_finite(v_vec, "v")
        u_norm: float = float(np.linalg.norm(u_vec))
        v_norm: float = float(np.linalg.norm(v_vec))
        if u_norm < eps or v_norm < eps:
            raise ValueError("u and v must be non-zero")
        cos_angle: float = float(np.dot(u_vec, v_vec) / (u_norm * v_norm))
        cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
        return float(np.arccos(cos_angle))


class Mat3:
    """Matrix utilities for 3x3 matrices."""

    @staticmethod
    def is_spd(A: NDArray[np.float64], tol: float = 1e-12) -> bool:
        """Check whether a matrix is symmetric positive definite."""
        mat: NDArray[np.float64] = np.asarray(A, dtype=float)
        if mat.shape != (3, 3):
            return False
        if not np.allclose(mat, mat.T, atol=tol):
            return False
        eigvals: NDArray[np.float64] = np.asarray(np.linalg.eigvalsh(mat), dtype=float)
        return bool(np.all(eigvals > tol))

    @staticmethod
    def sym(A: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return the symmetric part of a 3x3 matrix."""
        mat: NDArray[np.float64] = np.asarray(A, dtype=float)
        if mat.shape != (3, 3):
            raise ValueError("A must be shape (3, 3)")
        return 0.5 * (mat + mat.T)

    @staticmethod
    def clamp_spd(
        A: NDArray[np.float64],
        eig_min: float,
        eig_max: float,
    ) -> NDArray[np.float64]:
        """Clamp eigenvalues of a symmetric matrix and return SPD matrix."""
        mat: NDArray[np.float64] = np.asarray(A, dtype=float)
        if mat.shape != (3, 3):
            raise ValueError("A must be shape (3, 3)")
        assert_finite(mat, "A")
        if eig_min > eig_max:
            raise ValueError("eig_min must be <= eig_max")

        sym_mat: NDArray[np.float64] = Mat3.sym(mat)
        eigvals: NDArray[np.float64]
        eigvecs: NDArray[np.float64]
        eigvals, eigvecs = np.linalg.eigh(sym_mat)
        clamped: NDArray[np.float64] = np.clip(eigvals, eig_min, eig_max)
        diag: NDArray[np.float64] = np.diag(clamped)
        result: NDArray[np.float64] = eigvecs @ diag @ eigvecs.T
        return Mat3.sym(result)
