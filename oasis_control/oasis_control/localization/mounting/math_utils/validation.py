################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Validation helpers for mounting calibration inputs."""

from __future__ import annotations

from typing import Sequence

import numpy as np


# Quaternion normalization tolerance for near-zero checks
QUAT_NORM_EPS: float = 1e-12

# Covariance symmetry tolerance for soft symmetrization
COV_SYMMETRY_ATOL: float = 1e-8
COV_SYMMETRY_RTOL: float = 1e-5


def reshape_matrix(
    values: Sequence[float],
    shape: tuple[int, int],
    name: str,
) -> np.ndarray:
    """Return a finite matrix reshaped to the target shape."""
    array: np.ndarray = np.asarray(values, dtype=np.float64)
    if array.size != shape[0] * shape[1]:
        raise ValueError(f"{name} must have {shape[0] * shape[1]} elements")

    matrix: np.ndarray = array.reshape(shape)
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be finite")

    return matrix


def reshape_covariance(
    values: Sequence[float],
    shape: tuple[int, int],
    name: str,
) -> np.ndarray:
    """Return a finite covariance matrix with basic validation."""
    matrix: np.ndarray = reshape_matrix(values, shape, name)
    if np.all(matrix == -1.0) or np.all(np.diag(matrix) == -1.0):
        raise ValueError(f"{name} unknown (-1); full covariance required")

    if not np.allclose(
        matrix,
        matrix.T,
        atol=COV_SYMMETRY_ATOL,
        rtol=COV_SYMMETRY_RTOL,
    ):
        # Symmetrize slightly asymmetric covariances from drivers
        matrix = 0.5 * (matrix + matrix.T)

    if np.any(np.diag(matrix) < 0.0):
        raise ValueError(f"{name} diagonal must be non-negative")

    return matrix


def normalize_quaternion_wxyz(wxyz: Sequence[float]) -> np.ndarray:
    """Return a unit quaternion from wxyz input."""
    if len(wxyz) != 4:
        raise ValueError("Quaternion must have 4 elements")

    array: np.ndarray = np.asarray(wxyz, dtype=np.float64)
    if array.shape != (4,):
        array = array.reshape((4,))
    if not np.all(np.isfinite(array)):
        raise ValueError("Quaternion must be finite")

    norm: float = float(np.linalg.norm(array))
    if not np.isfinite(norm) or norm <= QUAT_NORM_EPS:
        raise ValueError("Quaternion norm must be positive")

    return array / norm
