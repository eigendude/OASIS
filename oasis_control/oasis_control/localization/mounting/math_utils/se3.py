################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""SE(3) rigid-body transforms."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .linalg import SO3
from .linalg import Linalg
from .quat import Quaternion
from .units import assert_finite


@dataclass(frozen=True)
class SE3:
    """Rigid-body transform with rotation and translation."""

    R: NDArray[np.float64]
    p: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate and project rotation and translation inputs."""
        R_mat: NDArray[np.float64] = np.asarray(self.R, dtype=float)
        p_vec: NDArray[np.float64] = np.asarray(self.p, dtype=float)
        Linalg.ensure_shape(R_mat, (3, 3), "R")
        Linalg.ensure_shape(p_vec, (3,), "p")
        assert_finite(R_mat, "R")
        assert_finite(p_vec, "p")
        R_proj: NDArray[np.float64] = SO3.project_to_so3(R_mat)
        object.__setattr__(self, "R", R_proj)
        object.__setattr__(self, "p", p_vec)

    @staticmethod
    def identity() -> "SE3":
        """Return the identity transform."""
        return SE3(np.eye(3, dtype=float), np.zeros(3, dtype=float))

    @staticmethod
    def from_quat_translation(q: Quaternion, p: NDArray[np.float64]) -> "SE3":
        """Create a transform from a quaternion and translation."""
        vec: NDArray[np.float64] = np.asarray(p, dtype=float)
        if vec.shape != (3,):
            raise ValueError("p must be shape (3,)")
        assert_finite(vec, "p")
        return SE3(q.as_matrix(), vec)

    def as_matrix4(self) -> NDArray[np.float64]:
        """Return the homogeneous 4x4 transform matrix."""
        mat: NDArray[np.float64] = np.eye(4, dtype=float)
        mat[:3, :3] = self.R
        mat[:3, 3] = self.p
        return mat

    def inverse(self) -> "SE3":
        """Return the inverse transform."""
        R_inv: NDArray[np.float64] = self.R.T
        p_inv: NDArray[np.float64] = -(R_inv @ self.p)
        return SE3(R_inv, p_inv)

    def __mul__(self, other: "SE3") -> "SE3":
        """Compose two transforms."""
        R_new: NDArray[np.float64] = self.R @ other.R
        p_new: NDArray[np.float64] = self.R @ other.p + self.p
        return SE3(R_new, p_new)

    def transform_point(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        """Transform a point by rotation and translation."""
        vec: NDArray[np.float64] = np.asarray(x, dtype=float)
        if vec.shape != (3,):
            raise ValueError("x must be shape (3,)")
        assert_finite(vec, "x")
        return self.R @ vec + self.p

    def transform_vector(self, v: NDArray[np.float64]) -> NDArray[np.float64]:
        """Transform a vector by rotation only."""
        vec: NDArray[np.float64] = np.asarray(v, dtype=float)
        if vec.shape != (3,):
            raise ValueError("v must be shape (3,)")
        assert_finite(vec, "v")
        return self.R @ vec
