################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Covariance container utilities for mounting calibration state."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Symmetry tolerance for covariance validation
SYM_TOL: float = 1e-9

# Default PSD tolerance for eigenvalue checks
PSD_TOL: float = 1e-12


class CovarianceError(Exception):
    """Raised when covariance matrices are invalid or unsupported."""


@dataclass(frozen=True)
class Covariance:
    """Container for symmetric covariance matrices.

    Attributes:
        P: Symmetric covariance matrix with shape (N, N)
    """

    P: np.ndarray

    def __post_init__(self) -> None:
        """Validate covariance shape, dtype, and symmetry."""
        P: np.ndarray = np.asarray(self.P, dtype=np.float64)
        if P.ndim != 2 or P.shape[0] != P.shape[1]:
            raise CovarianceError("Covariance must be a square matrix")
        if not np.all(np.isfinite(P)):
            raise CovarianceError("Covariance contains non-finite values")
        if not np.allclose(P, P.T, rtol=0.0, atol=SYM_TOL):
            raise CovarianceError("Covariance must be symmetric")
        object.__setattr__(self, "P", P)

    def dim(self) -> int:
        """Return the dimension of the covariance matrix."""
        return int(self.P.shape[0])

    def as_array(self) -> np.ndarray:
        """Return a defensive copy of the covariance matrix."""
        return self.P.copy()

    def symmetrized(self) -> Covariance:
        """Return a symmetrized version of this covariance matrix."""
        sym: np.ndarray = 0.5 * (self.P + self.P.T)
        return Covariance(sym)

    def is_psd(self, *, tol: float = PSD_TOL) -> bool:
        """Return True when the covariance is positive semi-definite."""
        eigvals: np.ndarray = np.linalg.eigvalsh(self.P)
        return bool(np.min(eigvals) >= -tol)

    def assert_psd(self, *, tol: float = PSD_TOL) -> None:
        """Raise CovarianceError if the covariance is not PSD."""
        if not self.is_psd(tol=tol):
            raise CovarianceError("Covariance is not positive semi-definite")

    @staticmethod
    def zeros(dim: int) -> Covariance:
        """Return a zero covariance matrix of the given dimension."""
        if dim <= 0:
            raise CovarianceError("dim must be positive")
        mat: np.ndarray = np.zeros((dim, dim), dtype=np.float64)
        return Covariance(mat)

    @staticmethod
    def eye(dim: int, *, scale: float = 1.0) -> Covariance:
        """Return a scaled identity covariance matrix."""
        if dim <= 0:
            raise CovarianceError("dim must be positive")
        mat: np.ndarray = np.eye(dim, dtype=np.float64) * float(scale)
        return Covariance(mat)
