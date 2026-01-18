################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Small linear-algebra helpers for AHRS core math

Matrices are represented as row-major lists of floats. Element (r, c) for a
matrix with ``cols`` columns is stored at ``data[r * cols + c]``. This matches
existing AHRS matrix conventions and keeps the math layer dependency-light.

Covariance matrices are expected to be symmetric positive (semi-)definite (SPD
or PSD). For AHRS updates, the innovation covariance ``S`` is the sum of the
predicted measurement covariance and the measurement noise covariance. The
Mahalanobis distance ``d^2 = nu^T S^{-1} nu`` is used for gating updates.
"""

from __future__ import annotations

import math
from collections.abc import Sequence


Matrix = list[float]
Vector = list[float]


def _validate_matrix_size(a: Matrix, rows: int, cols: int, name: str) -> None:
    if rows <= 0 or cols <= 0:
        raise ValueError(f"{name} rows and cols must be positive")
    expected: int = rows * cols
    if len(a) != expected:
        raise ValueError(f"{name} must have length {expected} for {rows}x{cols}")


def _validate_vector_size(x: Vector, size: int, name: str) -> None:
    if size <= 0:
        raise ValueError(f"{name} size must be positive")
    if len(x) != size:
        raise ValueError(f"{name} must have length {size}")


def _validate_mat3(a: Matrix, name: str) -> None:
    if len(a) != 9:
        raise ValueError(f"{name} must have length 9 for 3x3")


def _diag_only_from_matrix(a: Matrix, diag_min: float) -> Matrix:
    d0: float = max(a[0], diag_min)
    d1: float = max(a[4], diag_min)
    d2: float = max(a[8], diag_min)
    return [
        d0,
        0.0,
        0.0,
        0.0,
        d1,
        0.0,
        0.0,
        0.0,
        d2,
    ]


def mat_get(a: Matrix, rows: int, cols: int, r: int, c: int) -> float:
    """Return a matrix entry using row-major indexing.

    Args:
        a: Matrix in row-major form
        rows: Number of rows in the matrix
        cols: Number of columns in the matrix
        r: Row index
        c: Column index

    Returns:
        Matrix value at (r, c)

    Raises:
        ValueError: If size or indices are invalid
    """
    _validate_matrix_size(a, rows, cols, "a")
    if r < 0 or r >= rows or c < 0 or c >= cols:
        raise ValueError("row or column index out of range")
    return a[r * cols + c]


def mat_set(a: Matrix, rows: int, cols: int, r: int, c: int, v: float) -> None:
    """Set a matrix entry using row-major indexing.

    Args:
        a: Matrix in row-major form
        rows: Number of rows in the matrix
        cols: Number of columns in the matrix
        r: Row index
        c: Column index
        v: Value to set

    Raises:
        ValueError: If size or indices are invalid
    """
    _validate_matrix_size(a, rows, cols, "a")
    if r < 0 or r >= rows or c < 0 or c >= cols:
        raise ValueError("row or column index out of range")
    a[r * cols + c] = v


def mat_transpose(a: Matrix, rows: int, cols: int) -> Matrix:
    """Return the transpose of a matrix.

    Args:
        a: Matrix in row-major form
        rows: Number of rows in the matrix
        cols: Number of columns in the matrix

    Returns:
        Transposed matrix with shape (cols, rows)

    Raises:
        ValueError: If the input size is invalid
    """
    _validate_matrix_size(a, rows, cols, "a")
    out: Matrix = [0.0] * (rows * cols)
    for r in range(rows):
        for c in range(cols):
            out[c * rows + r] = a[r * cols + c]
    return out


def mat_mul(
    a: Matrix,
    a_rows: int,
    a_cols: int,
    b: Matrix,
    b_rows: int,
    b_cols: int,
) -> Matrix:
    """Multiply two matrices.

    Args:
        a: Left matrix in row-major form
        a_rows: Number of rows in ``a``
        a_cols: Number of columns in ``a``
        b: Right matrix in row-major form
        b_rows: Number of rows in ``b``
        b_cols: Number of columns in ``b``

    Returns:
        Matrix product with shape (a_rows, b_cols)

    Raises:
        ValueError: If sizes are invalid or inner dimensions mismatch
    """
    _validate_matrix_size(a, a_rows, a_cols, "a")
    _validate_matrix_size(b, b_rows, b_cols, "b")
    if a_cols != b_rows:
        raise ValueError("a_cols must match b_rows")
    out: Matrix = [0.0] * (a_rows * b_cols)
    for r in range(a_rows):
        row_base: int = r * a_cols
        for c in range(b_cols):
            total: float = 0.0
            for k in range(a_cols):
                total += a[row_base + k] * b[k * b_cols + c]
            out[r * b_cols + c] = total
    return out


def mat_vec_mul(a: Matrix, rows: int, cols: int, x: Vector) -> Vector:
    """Multiply a matrix by a vector.

    Args:
        a: Matrix in row-major form
        rows: Number of rows in the matrix
        cols: Number of columns in the matrix
        x: Vector with length ``cols``

    Returns:
        Vector with length ``rows``

    Raises:
        ValueError: If sizes are invalid
    """
    _validate_matrix_size(a, rows, cols, "a")
    _validate_vector_size(x, cols, "x")
    out: Vector = [0.0] * rows
    for r in range(rows):
        row_base: int = r * cols
        total: float = 0.0
        for c in range(cols):
            total += a[row_base + c] * x[c]
        out[r] = total
    return out


def mat_add(a: Matrix, b: Matrix) -> Matrix:
    """Add two matrices of the same size.

    Args:
        a: Left matrix in row-major form
        b: Right matrix in row-major form

    Returns:
        Element-wise sum of ``a`` and ``b``

    Raises:
        ValueError: If sizes do not match
    """
    if len(a) != len(b):
        raise ValueError("matrices must have the same length")
    return [ai + bi for ai, bi in zip(a, b, strict=True)]


def mat_sub(a: Matrix, b: Matrix) -> Matrix:
    """Subtract two matrices of the same size.

    Args:
        a: Left matrix in row-major form
        b: Right matrix in row-major form

    Returns:
        Element-wise difference ``a - b``

    Raises:
        ValueError: If sizes do not match
    """
    if len(a) != len(b):
        raise ValueError("matrices must have the same length")
    return [ai - bi for ai, bi in zip(a, b, strict=True)]


def mat_scale(a: Matrix, s: float) -> Matrix:
    """Scale a matrix by a scalar.

    Args:
        a: Matrix in row-major form
        s: Scalar multiplier

    Returns:
        Scaled matrix
    """
    return [ai * s for ai in a]


def symmetrize(a: Matrix, n: int) -> Matrix:
    """Return the symmetric part of a square matrix.

    Symmetry is expected for covariance matrices and enforcing it reduces
    numerical drift from repeated updates.

    Args:
        a: Matrix in row-major form
        n: Matrix dimension

    Returns:
        Symmetric matrix ``0.5 * (A + A^T)``

    Raises:
        ValueError: If size is invalid
    """
    _validate_matrix_size(a, n, n, "a")
    out: Matrix = [0.0] * (n * n)
    for r in range(n):
        for c in range(n):
            # 0.5 is the average of A and A^T
            out[r * n + c] = 0.5 * (a[r * n + c] + a[c * n + r])
    return out


def is_finite_seq(values: Sequence[float]) -> bool:
    """Return True when all values are finite.

    Args:
        values: Sequence to validate

    Returns:
        True if every element is finite
    """
    return all(math.isfinite(value) for value in values)


def mat3_det(a: Matrix) -> float:
    """Return the determinant of a 3x3 matrix.

    Args:
        a: 3x3 matrix in row-major form

    Returns:
        Determinant of ``a``

    Raises:
        ValueError: If the matrix size is invalid
    """
    _validate_mat3(a, "a")
    a00: float = a[0]
    a01: float = a[1]
    a02: float = a[2]
    a10: float = a[3]
    a11: float = a[4]
    a12: float = a[5]
    a20: float = a[6]
    a21: float = a[7]
    a22: float = a[8]
    return (
        a00 * (a11 * a22 - a12 * a21)
        - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20)
    )


def mat3_inv(a: Matrix) -> Matrix:
    """Invert a 3x3 matrix for innovation covariance solves.

    Args:
        a: 3x3 matrix in row-major form

    Returns:
        Inverse of ``a``

    Raises:
        ValueError: If the matrix is singular or nearly singular
    """
    _validate_mat3(a, "a")
    det: float = mat3_det(a)
    # 1e-12 is the determinant threshold for near-singular matrices
    if abs(det) < 1.0e-12:
        raise ValueError("matrix is singular or nearly singular")

    a00: float = a[0]
    a01: float = a[1]
    a02: float = a[2]
    a10: float = a[3]
    a11: float = a[4]
    a12: float = a[5]
    a20: float = a[6]
    a21: float = a[7]
    a22: float = a[8]

    inv_det: float = 1.0 / det

    return [
        (a11 * a22 - a12 * a21) * inv_det,
        (a02 * a21 - a01 * a22) * inv_det,
        (a01 * a12 - a02 * a11) * inv_det,
        (a12 * a20 - a10 * a22) * inv_det,
        (a00 * a22 - a02 * a20) * inv_det,
        (a02 * a10 - a00 * a12) * inv_det,
        (a10 * a21 - a11 * a20) * inv_det,
        (a01 * a20 - a00 * a21) * inv_det,
        (a00 * a11 - a01 * a10) * inv_det,
    ]


def mat3_solve(a: Matrix, b: Vector) -> Vector:
    """Solve a 3x3 linear system ``A x = b``.

    This uses a direct inverse, which is acceptable for small 3x3 systems. A
    future implementation may replace this with a Cholesky-based solver.

    Args:
        a: 3x3 matrix in row-major form
        b: Right-hand vector with length 3

    Returns:
        Solution vector ``x`` with length 3

    Raises:
        ValueError: If the matrix or vector sizes are invalid
    """
    _validate_mat3(a, "a")
    _validate_vector_size(b, 3, "b")
    inv: Matrix = mat3_inv(a)
    return mat_vec_mul(inv, 3, 3, b)


def quad_form_3(x: Vector, a: Matrix) -> float:
    """Return the quadratic form ``x^T A x`` for 3D vectors.

    Args:
        x: Vector with length 3
        a: 3x3 matrix in row-major form

    Returns:
        Scalar quadratic form

    Raises:
        ValueError: If sizes are invalid
    """
    _validate_vector_size(x, 3, "x")
    _validate_mat3(a, "a")
    ax: Vector = mat_vec_mul(a, 3, 3, x)
    return x[0] * ax[0] + x[1] * ax[1] + x[2] * ax[2]


def clamp_diag_3(a: Matrix, diag_min: float, diag_max: float) -> Matrix:
    """Clamp the diagonal of a 3x3 matrix.

    Off-diagonal terms are preserved. This is a weak safeguard and does not
    guarantee positive definiteness.

    Args:
        a: 3x3 matrix in row-major form
        diag_min: Minimum value for diagonal entries
        diag_max: Maximum value for diagonal entries

    Returns:
        Matrix with clamped diagonal entries

    Raises:
        ValueError: If sizes are invalid or clamp bounds are invalid
    """
    _validate_mat3(a, "a")
    if diag_max < diag_min:
        raise ValueError("diag_max must be >= diag_min")
    out: Matrix = list(a)
    out[0] = min(max(out[0], diag_min), diag_max)
    out[4] = min(max(out[4], diag_min), diag_max)
    out[8] = min(max(out[8], diag_min), diag_max)
    return out


def project_to_symmetric_psd_3(a: Matrix, diag_min: float) -> Matrix:
    """Project a 3x3 matrix to a symmetric PSD-like matrix.

    This is a pragmatic approximation used for adaptive covariance updates. The
    matrix is symmetrized, diagonal entries are clamped, and if the determinant
    is negative or the inverse fails, the result falls back to a diagonal-only
    matrix.

    Args:
        a: 3x3 matrix in row-major form
        diag_min: Minimum value for diagonal entries

    Returns:
        Symmetric, PSD-like matrix

    Raises:
        ValueError: If size is invalid
    """
    _validate_mat3(a, "a")
    symmetric: Matrix = symmetrize(a, 3)
    clamped: Matrix = clamp_diag_3(symmetric, diag_min, math.inf)
    det: float = mat3_det(clamped)
    if det < 0.0:
        return _diag_only_from_matrix(clamped, diag_min)
    try:
        mat3_inv(clamped)
    except ValueError:
        return _diag_only_from_matrix(clamped, diag_min)
    return clamped


def innovation_covariance(s_hat: Matrix, r: Matrix) -> Matrix:
    """Return innovation covariance ``S = S_hat + R``.

    Args:
        s_hat: Predicted measurement covariance in row-major form
        r: Measurement noise covariance in row-major form

    Returns:
        Innovation covariance in row-major form

    Raises:
        ValueError: If sizes do not match
    """
    return mat_add(s_hat, r)


def mahalanobis_d2_3(nu: Vector, s: Matrix) -> float:
    """Return the squared Mahalanobis distance for a 3D innovation.

    Args:
        nu: Innovation vector with length 3
        s: Innovation covariance matrix (3x3)

    Returns:
        Squared Mahalanobis distance ``nu^T S^{-1} nu``

    Raises:
        ValueError: If sizes are invalid or ``S`` is singular
    """
    _validate_vector_size(nu, 3, "nu")
    _validate_mat3(s, "s")
    sol: Vector = mat3_solve(s, nu)
    return nu[0] * sol[0] + nu[1] * sol[1] + nu[2] * sol[2]
