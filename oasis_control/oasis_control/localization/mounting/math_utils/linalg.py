################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Linear algebra utilities for rotations and matrices."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .units import PhysicalConstants
from .units import assert_finite


class SO3:
    """SO(3) rotation utilities."""

    @staticmethod
    def hat(w: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return the skew-symmetric matrix for a rotation vector."""
        vec: NDArray[np.float64] = np.asarray(w, dtype=float)
        Linalg.ensure_shape(vec, (3,), "w")
        assert_finite(vec, "w")
        wx: float = float(vec[0])
        wy: float = float(vec[1])
        wz: float = float(vec[2])
        return np.array(
            [
                [0.0, -wz, wy],
                [wz, 0.0, -wx],
                [-wy, wx, 0.0],
            ],
            dtype=float,
        )

    @staticmethod
    def vee(W: NDArray[np.float64]) -> NDArray[np.float64]:
        """Return the rotation vector from a skew-symmetric matrix."""
        mat: NDArray[np.float64] = np.asarray(W, dtype=float)
        Linalg.ensure_shape(mat, (3, 3), "W")
        assert_finite(mat, "W")
        return np.array([mat[2, 1], mat[0, 2], mat[1, 0]], dtype=float)

    @staticmethod
    def exp(w: NDArray[np.float64]) -> NDArray[np.float64]:
        """Exponentiate a rotation vector to a rotation matrix."""
        vec: NDArray[np.float64] = np.asarray(w, dtype=float)
        Linalg.ensure_shape(vec, (3,), "w")
        assert_finite(vec, "w")
        theta: float = float(np.linalg.norm(vec))
        W: NDArray[np.float64] = SO3.hat(vec)
        eye: NDArray[np.float64] = np.eye(3, dtype=float)
        if theta < 1e-8:
            return eye + W + 0.5 * (W @ W)
        sin_theta: float = float(np.sin(theta))
        cos_theta: float = float(np.cos(theta))
        A: float = sin_theta / theta
        B: float = (1.0 - cos_theta) / (theta * theta)
        return eye + A * W + B * (W @ W)

    @staticmethod
    def log(R: NDArray[np.float64]) -> NDArray[np.float64]:
        """Compute the rotation vector from a rotation matrix."""
        mat: NDArray[np.float64] = np.asarray(R, dtype=float)
        Linalg.ensure_shape(mat, (3, 3), "R")
        assert_finite(mat, "R")
        cos_theta: float = float((np.trace(mat) - 1.0) * 0.5)
        cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
        theta: float = float(np.arccos(cos_theta))
        if theta < 1e-8:
            return 0.5 * SO3.vee(mat - mat.T)
        sin_theta: float = float(np.sin(theta))
        if abs(sin_theta) < PhysicalConstants.EPS:
            return 0.5 * SO3.vee(mat - mat.T)
        scale: float = theta / (2.0 * sin_theta)
        return scale * SO3.vee(mat - mat.T)

    @staticmethod
    def project_to_so3(R: NDArray[np.float64]) -> NDArray[np.float64]:
        """Project a matrix to the nearest SO(3) rotation matrix."""
        mat: NDArray[np.float64] = np.asarray(R, dtype=float)
        Linalg.ensure_shape(mat, (3, 3), "R")
        assert_finite(mat, "R")
        U: NDArray[np.float64]
        S: NDArray[np.float64]
        Vt: NDArray[np.float64]
        U, S, Vt = np.linalg.svd(mat)
        R_proj: NDArray[np.float64] = U @ Vt
        if np.linalg.det(R_proj) < 0.0:
            U[:, -1] *= -1.0
            R_proj = U @ Vt
        return R_proj

    @staticmethod
    def angle(R: NDArray[np.float64]) -> float:
        """Return the rotation angle of a rotation matrix."""
        mat: NDArray[np.float64] = np.asarray(R, dtype=float)
        Linalg.ensure_shape(mat, (3, 3), "R")
        assert_finite(mat, "R")
        cos_theta: float = float((np.trace(mat) - 1.0) * 0.5)
        cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
        return float(np.arccos(cos_theta))


class Linalg:
    """General linear algebra helpers."""

    @staticmethod
    def block_diag(*blocks: NDArray[np.float64]) -> NDArray[np.float64]:
        """Create a block diagonal matrix from the given blocks."""
        if not blocks:
            return np.zeros((0, 0), dtype=float)
        shapes: list[tuple[int, int]] = []
        for block in blocks:
            mat: NDArray[np.float64] = np.asarray(block, dtype=float)
            if mat.ndim != 2:
                raise ValueError("blocks must be 2D arrays")
            shapes.append((mat.shape[0], mat.shape[1]))
        total_rows: int = sum(shape[0] for shape in shapes)
        total_cols: int = sum(shape[1] for shape in shapes)
        result: NDArray[np.float64] = np.zeros((total_rows, total_cols), dtype=float)
        row: int = 0
        col: int = 0
        for block, (rows, cols) in zip(blocks, shapes):
            mat_block: NDArray[np.float64] = np.asarray(block, dtype=float)
            result[row : row + rows, col : col + cols] = mat_block
            row += rows
            col += cols
        return result

    @staticmethod
    def ensure_shape(x: NDArray[np.float64], shape: tuple[int, ...], name: str) -> None:
        """Ensure an array has the expected shape."""
        if x.shape != shape:
            raise ValueError(f"{name} must have shape {shape}")
