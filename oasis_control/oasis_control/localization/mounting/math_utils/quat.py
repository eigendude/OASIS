################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
"""Quaternion utilities using the wxyz convention."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .linalg import SO3
from .units import PhysicalConstants
from .units import assert_finite


@dataclass(frozen=True)
class Quaternion:
    """Quaternion stored in wxyz order."""

    wxyz: NDArray[np.float64]

    def __post_init__(self) -> None:
        """Validate quaternion inputs and normalize storage."""
        wxyz: NDArray[np.float64] = np.asarray(self.wxyz, dtype=float)
        if wxyz.shape != (4,):
            raise ValueError("wxyz must be shape (4,)")
        assert_finite(wxyz, "wxyz")
        object.__setattr__(self, "wxyz", wxyz)

    @staticmethod
    def identity() -> "Quaternion":
        """Return the identity quaternion."""
        return Quaternion.from_wxyz(1.0, 0.0, 0.0, 0.0)

    @staticmethod
    def from_wxyz(w: float, x: float, y: float, z: float) -> "Quaternion":
        """Create a quaternion from components."""
        return Quaternion(np.array([w, x, y, z], dtype=float))

    @staticmethod
    def from_rotvec(w: NDArray[np.float64]) -> "Quaternion":
        """Create a quaternion from a rotation vector."""
        R: NDArray[np.float64] = SO3.exp(w)
        return Quaternion.from_matrix(R)

    @staticmethod
    def from_matrix(R: NDArray[np.float64]) -> "Quaternion":
        """Create a quaternion from a rotation matrix."""
        mat: NDArray[np.float64] = np.asarray(R, dtype=float)
        if mat.shape != (3, 3):
            raise ValueError("R must be shape (3, 3)")
        assert_finite(mat, "R")
        trace: float = float(np.trace(mat))
        if trace > 0.0:
            s: float = float(np.sqrt(trace + 1.0) * 2.0)
            w: float = 0.25 * s
            x: float = float((mat[2, 1] - mat[1, 2]) / s)
            y: float = float((mat[0, 2] - mat[2, 0]) / s)
            z: float = float((mat[1, 0] - mat[0, 1]) / s)
        else:
            diag: NDArray[np.float64] = np.diag(mat)
            idx: int = int(np.argmax(diag))
            if idx == 0:
                s = float(np.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2]) * 2.0)
                w = float((mat[2, 1] - mat[1, 2]) / s)
                x = 0.25 * s
                y = float((mat[0, 1] + mat[1, 0]) / s)
                z = float((mat[0, 2] + mat[2, 0]) / s)
            elif idx == 1:
                s = float(np.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2]) * 2.0)
                w = float((mat[0, 2] - mat[2, 0]) / s)
                x = float((mat[0, 1] + mat[1, 0]) / s)
                y = 0.25 * s
                z = float((mat[1, 2] + mat[2, 1]) / s)
            else:
                s = float(np.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1]) * 2.0)
                w = float((mat[1, 0] - mat[0, 1]) / s)
                x = float((mat[0, 2] + mat[2, 0]) / s)
                y = float((mat[1, 2] + mat[2, 1]) / s)
                z = 0.25 * s
        return Quaternion.from_wxyz(w, x, y, z).normalized()

    def as_matrix(self) -> NDArray[np.float64]:
        """Return the rotation matrix representation."""
        q: NDArray[np.float64] = self.normalized().wxyz
        w: float = float(q[0])
        x: float = float(q[1])
        y: float = float(q[2])
        z: float = float(q[3])
        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=float,
        )

    def normalized(self) -> "Quaternion":
        """Return a normalized quaternion."""
        norm: float = float(np.linalg.norm(self.wxyz))
        if norm < PhysicalConstants.EPS:
            raise ValueError("Quaternion norm is too small")
        return Quaternion(self.wxyz / norm)

    def inverse(self) -> "Quaternion":
        """Return the inverse quaternion."""
        q: NDArray[np.float64] = self.normalized().wxyz
        return Quaternion.from_wxyz(
            float(q[0]), float(-q[1]), float(-q[2]), float(-q[3])
        )

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        """Multiply two quaternions using the Hamilton product."""
        q1: NDArray[np.float64] = self.wxyz
        q2: NDArray[np.float64] = other.wxyz
        w1: float = float(q1[0])
        x1: float = float(q1[1])
        y1: float = float(q1[2])
        z1: float = float(q1[3])
        w2: float = float(q2[0])
        x2: float = float(q2[1])
        y2: float = float(q2[2])
        z2: float = float(q2[3])
        w: float = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x: float = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y: float = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z: float = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return Quaternion.from_wxyz(w, x, y, z)

    def rotate(self, v: NDArray[np.float64]) -> NDArray[np.float64]:
        """Rotate a 3-vector by this quaternion."""
        vec: NDArray[np.float64] = np.asarray(v, dtype=float)
        if vec.shape != (3,):
            raise ValueError("v must be shape (3,)")
        assert_finite(vec, "v")
        return self.as_matrix() @ vec

    def to_wxyz(self) -> NDArray[np.float64]:
        """Return a copy of the quaternion components."""
        return np.array(self.wxyz, dtype=float)

    def almost_equal(self, other: "Quaternion", atol: float = 1e-9) -> bool:
        """Check approximate equality, accounting for sign ambiguity."""
        q1: NDArray[np.float64] = self.wxyz
        q2: NDArray[np.float64] = other.wxyz
        if np.allclose(q1, q2, atol=atol):
            return True
        return bool(np.allclose(q1, -q2, atol=atol))
