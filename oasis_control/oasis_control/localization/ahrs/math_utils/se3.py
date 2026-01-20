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

from math import acos, cos, sin, sqrt
from typing import List, Sequence, Tuple


class Se3:
    """SE(3) utilities for AHRS extrinsics and perturbations.

    Responsibility:
        Define rigid-body transforms and adjoint mappings for extrinsics used
        in the AHRS state and error-state models.

    Purpose:
        Provide the rigid-body transform conventions and operators needed to
        express IMU/magnetometer extrinsics in a ROS-agnostic core.

    Inputs/outputs:
        - Transform T_AB = (R_AB, p_AB) maps vectors from {B} to {A}.
        - Rotation matrices are 3x3, translations are 3x1.
        - Tangent vectors δξ are 6x1 with [δρ; δθ].

    Dependencies:
        - Used by extrinsics_model, state_mapping, and covariance transforms.
        - Works with Quaternion and LinearAlgebra utilities for consistency.

    Public API (to be implemented):
        - compose(T_ab, T_bc)
        - inverse(T_ab)
        - apply(T_ab, p_b)
        - to_matrix(T_ab)
        - from_matrix(T_ab_matrix)
        - exp(delta_xi)
        - log(T_ab)
        - hat(delta_xi)
        - vee(xi_hat)
        - adjoint(T_ab)

    Data contract:
        - R is a 3x3 rotation matrix.
        - p is a 3x1 translation vector in meters.
        - delta_xi is a 6x1 vector [delta_rho; delta_theta].

    Frames and units:
        - T_AB maps p_B to p_A via p_A = R_AB * p_B + p_AB.
        - delta_rho in meters, delta_theta in radians.

    Determinism and edge cases:
        - Transform composition and adjoint calculations are deterministic
          and numerically stable for small perturbations.
        - exp/log must be consistent for small-angle perturbations.
        - adjoint must be defined for any valid T_AB.

    Equations:
        Perturbation composition:
            T = Exp(δξ) * T_hat

        Adjoint for covariance mapping:
            Ad_T = [[R, [p]× R], [0, R]]
            Σ_A = Ad_T * Σ_B * Ad_Tᵀ

    Numerical stability notes:
        - Use series expansions for small rotation angles.
        - Avoid catastrophic cancellation when ||delta_theta|| is small.

    Suggested unit tests:
        - compose(inverse(T), T) yields identity.
        - exp(log(T)) returns T within tolerance.
        - adjoint matches finite-difference covariance mapping.
    """

    @staticmethod
    def compose(
        T_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
        T_bc: Tuple[Sequence[Sequence[float]], Sequence[float]],
    ) -> Tuple[List[List[float]], List[float]]:
        """Compose two transforms T_ab and T_bc into T_ac."""
        R_ab, p_ab = T_ab
        R_bc, p_bc = T_bc
        R_ac: List[List[float]] = _matmul3(R_ab, R_bc)
        p_ac: List[float] = _add3(_matvec3(R_ab, p_bc), p_ab)
        return (R_ac, p_ac)

    @staticmethod
    def inverse(
        T_ab: Tuple[Sequence[Sequence[float]], Sequence[float]]
    ) -> Tuple[List[List[float]], List[float]]:
        """Return inverse transform T_ba."""
        R_ab, p_ab = T_ab
        R_ba: List[List[float]] = _transpose3(R_ab)
        p_ba: List[float] = _scale3(_matvec3(R_ba, p_ab), -1.0)
        return (R_ba, p_ba)

    @staticmethod
    def apply(
        T_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
        p_b: Sequence[float],
    ) -> List[float]:
        """Apply transform to a point."""
        R_ab, p_ab = T_ab
        return _add3(_matvec3(R_ab, p_b), p_ab)

    @staticmethod
    def to_matrix(
        T_ab: Tuple[Sequence[Sequence[float]], Sequence[float]]
    ) -> List[List[float]]:
        """Return 4x4 homogeneous matrix representation."""
        R_ab, p_ab = T_ab
        if len(R_ab) != 3 or any(len(row) != 3 for row in R_ab):
            raise ValueError("Rotation matrix must be 3x3")
        if len(p_ab) != 3:
            raise ValueError("Translation must have length 3")
        return [
            [R_ab[0][0], R_ab[0][1], R_ab[0][2], p_ab[0]],
            [R_ab[1][0], R_ab[1][1], R_ab[1][2], p_ab[1]],
            [R_ab[2][0], R_ab[2][1], R_ab[2][2], p_ab[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def from_matrix(
        T: Sequence[Sequence[float]],
        tol: float = 1e-6,
    ) -> Tuple[List[List[float]], List[float]]:
        """Parse a 4x4 homogeneous matrix into (R, p)."""
        if len(T) != 4 or any(len(row) != 4 for row in T):
            raise ValueError("Transform must be 4x4")
        # tol is the tolerance for the fixed last row [0, 0, 0, 1]
        if (
            abs(T[3][0]) > tol
            or abs(T[3][1]) > tol
            or abs(T[3][2]) > tol
            or abs(T[3][3] - 1.0) > tol
        ):
            raise ValueError("Transform last row must be [0, 0, 0, 1]")
        R_ab: List[List[float]] = [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]],
        ]
        p_ab: List[float] = [T[0][3], T[1][3], T[2][3]]
        return (R_ab, p_ab)

    @staticmethod
    def hat(delta_xi: Sequence[float]) -> List[List[float]]:
        """Return the se(3) matrix (hat) for delta_xi."""
        if len(delta_xi) != 6:
            raise ValueError("delta_xi must have length 6")
        v = delta_xi[:3]
        w = delta_xi[3:]
        w_hat: List[List[float]] = _skew(w)
        return [
            [w_hat[0][0], w_hat[0][1], w_hat[0][2], v[0]],
            [w_hat[1][0], w_hat[1][1], w_hat[1][2], v[1]],
            [w_hat[2][0], w_hat[2][1], w_hat[2][2], v[2]],
            [0.0, 0.0, 0.0, 0.0],
        ]

    @staticmethod
    def vee(xi_hat: Sequence[Sequence[float]]) -> List[float]:
        """Return the vector (vee) for a se(3) matrix."""
        if len(xi_hat) != 4 or any(len(row) != 4 for row in xi_hat):
            raise ValueError("xi_hat must be 4x4")
        return [
            xi_hat[0][3],
            xi_hat[1][3],
            xi_hat[2][3],
            xi_hat[2][1],
            xi_hat[0][2],
            xi_hat[1][0],
        ]

    @staticmethod
    def exp(delta_xi: Sequence[float]) -> Tuple[List[List[float]], List[float]]:
        """Compute the SE(3) exponential map."""
        if len(delta_xi) != 6:
            raise ValueError("delta_xi must have length 6")
        rho = delta_xi[:3]
        theta = delta_xi[3:]
        R = _so3_exp(theta)
        V = _so3_v_matrix(theta)
        p = _matvec3(V, rho)
        return (R, p)

    @staticmethod
    def log(
        T_ab: Tuple[Sequence[Sequence[float]], Sequence[float]]
    ) -> List[float]:
        """Compute the SE(3) logarithm map."""
        R_ab, p_ab = T_ab
        theta = _so3_log(R_ab)
        V_inv = _so3_v_inv_matrix(theta)
        rho = _matvec3(V_inv, p_ab)
        return [rho[0], rho[1], rho[2], theta[0], theta[1], theta[2]]

    @staticmethod
    def adjoint(
        T_ab: Tuple[Sequence[Sequence[float]], Sequence[float]]
    ) -> List[List[float]]:
        """Return the 6x6 adjoint matrix for T_ab."""
        R_ab, p_ab = T_ab
        if len(R_ab) != 3 or any(len(row) != 3 for row in R_ab):
            raise ValueError("Rotation matrix must be 3x3")
        if len(p_ab) != 3:
            raise ValueError("Translation must have length 3")
        p_hat = _skew(p_ab)
        p_hat_r = _matmul3(p_hat, R_ab)
        zero: List[float] = [0.0, 0.0, 0.0]
        return [
            [
                R_ab[0][0],
                R_ab[0][1],
                R_ab[0][2],
                p_hat_r[0][0],
                p_hat_r[0][1],
                p_hat_r[0][2],
            ],
            [
                R_ab[1][0],
                R_ab[1][1],
                R_ab[1][2],
                p_hat_r[1][0],
                p_hat_r[1][1],
                p_hat_r[1][2],
            ],
            [
                R_ab[2][0],
                R_ab[2][1],
                R_ab[2][2],
                p_hat_r[2][0],
                p_hat_r[2][1],
                p_hat_r[2][2],
            ],
            [zero[0], zero[1], zero[2], R_ab[0][0], R_ab[0][1], R_ab[0][2]],
            [zero[0], zero[1], zero[2], R_ab[1][0], R_ab[1][1], R_ab[1][2]],
            [zero[0], zero[1], zero[2], R_ab[2][0], R_ab[2][1], R_ab[2][2]],
        ]


def _skew(v: Sequence[float]) -> List[List[float]]:
    """Return the 3x3 skew-symmetric matrix for v."""
    if len(v) != 3:
        raise ValueError("Vector must have length 3")
    return [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]


def _matmul3(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Multiply two 3x3 matrices."""
    if len(A) != 3 or len(B) != 3:
        raise ValueError("Matrices must be 3x3")
    if any(len(row) != 3 for row in A) or any(len(row) != 3 for row in B):
        raise ValueError("Matrices must be 3x3")
    result: List[List[float]] = [[0.0 for _ in range(3)] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            acc: float = 0.0
            for k in range(3):
                acc += A[i][k] * B[k][j]
            result[i][j] = acc
    return result


def _matvec3(A: Sequence[Sequence[float]], v: Sequence[float]) -> List[float]:
    """Multiply a 3x3 matrix with a 3-vector."""
    if len(v) != 3:
        raise ValueError("Vector must have length 3")
    if len(A) != 3 or any(len(row) != 3 for row in A):
        raise ValueError("Matrix must be 3x3")
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def _transpose3(A: Sequence[Sequence[float]]) -> List[List[float]]:
    """Return the transpose of a 3x3 matrix."""
    if len(A) != 3 or any(len(row) != 3 for row in A):
        raise ValueError("Matrix must be 3x3")
    return [
        [A[0][0], A[1][0], A[2][0]],
        [A[0][1], A[1][1], A[2][1]],
        [A[0][2], A[1][2], A[2][2]],
    ]


def _add3(a: Sequence[float], b: Sequence[float]) -> List[float]:
    """Add two 3-vectors."""
    if len(a) != 3 or len(b) != 3:
        raise ValueError("Vectors must have length 3")
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _scale3(v: Sequence[float], scale: float) -> List[float]:
    """Scale a 3-vector."""
    if len(v) != 3:
        raise ValueError("Vector must have length 3")
    return [v[0] * scale, v[1] * scale, v[2] * scale]


def _so3_exp(theta: Sequence[float], eps: float = 1e-8) -> List[List[float]]:
    """Exponential map from so(3) to SO(3)."""
    if len(theta) != 3:
        raise ValueError("theta must have length 3")
    angle: float = sqrt(theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2])
    K: List[List[float]] = _skew(theta)
    if angle < eps:
        angle_sq: float = angle * angle
        if angle_sq == 0.0:
            return _identity3()
        # sin(angle) / angle series: 1 - angle^2 / 6
        sin_term: float = 1.0 - angle_sq / 6.0
        # (1 - cos(angle)) / angle^2 series: 1/2 - angle^2 / 24
        cos_term: float = 0.5 - angle_sq / 24.0
        K2 = _matmul3(K, K)
        return _mat_add3(
            _mat_add3(_identity3(), _scale_mat3(K, sin_term)),
            _scale_mat3(K2, cos_term),
        )
    axis: List[float] = [theta[0] / angle, theta[1] / angle, theta[2] / angle]
    K_axis: List[List[float]] = _skew(axis)
    K2_axis = _matmul3(K_axis, K_axis)
    return _mat_add3(
        _mat_add3(_identity3(), _scale_mat3(K_axis, sin(angle))),
        _scale_mat3(K2_axis, 1.0 - cos(angle)),
    )


def _so3_log(R: Sequence[Sequence[float]], eps: float = 1e-8) -> List[float]:
    """Logarithm map from SO(3) to so(3)."""
    if len(R) != 3 or any(len(row) != 3 for row in R):
        raise ValueError("Rotation matrix must be 3x3")
    trace: float = R[0][0] + R[1][1] + R[2][2]
    # 0.5 scales the trace to the cosine of the rotation angle
    cos_angle: float = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    angle: float = acos(cos_angle)
    if angle < eps:
        return [0.0, 0.0, 0.0]
    # The 1e-5 threshold separates the pi-angle branch for stability
    if abs(angle - 3.141592653589793) < 1e-5:
        # 0.5 scales the diagonal terms from (R + I) for the axis direction
        axis: List[float] = [
            sqrt(max(0.0, (R[0][0] + 1.0) * 0.5)),
            sqrt(max(0.0, (R[1][1] + 1.0) * 0.5)),
            sqrt(max(0.0, (R[2][2] + 1.0) * 0.5)),
        ]
        if R[2][1] - R[1][2] < 0.0:
            axis[0] = -axis[0]
        if R[0][2] - R[2][0] < 0.0:
            axis[1] = -axis[1]
        if R[1][0] - R[0][1] < 0.0:
            axis[2] = -axis[2]
        return [axis[0] * angle, axis[1] * angle, axis[2] * angle]
    # scale = angle / (2 * sin(angle)) converts skew part to axis-angle
    scale: float = angle / (2.0 * sin(angle))
    return [
        scale * (R[2][1] - R[1][2]),
        scale * (R[0][2] - R[2][0]),
        scale * (R[1][0] - R[0][1]),
    ]


def _so3_v_matrix(theta: Sequence[float], eps: float = 1e-8) -> List[List[float]]:
    """Return the V matrix used in SE(3) exponential."""
    angle: float = sqrt(theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2])
    K: List[List[float]] = _skew(theta)
    if angle < eps:
        angle_sq: float = angle * angle
        if angle_sq == 0.0:
            return _identity3()
        # 0.5 and 1/6 are the first-order series coefficients for V
        term1: float = 0.5 - angle_sq / 24.0
        term2: float = 1.0 / 6.0 - angle_sq / 120.0
        K2 = _matmul3(K, K)
        return _mat_add3(
            _identity3(),
            _mat_add3(_scale_mat3(K, term1), _scale_mat3(K2, term2)),
        )
    K2 = _matmul3(K, K)
    return _mat_add3(
        _identity3(),
        _mat_add3(
            _scale_mat3(K, (1.0 - cos(angle)) / (angle * angle)),
            _scale_mat3(K2, (angle - sin(angle)) / (angle * angle * angle)),
        ),
    )


def _so3_v_inv_matrix(theta: Sequence[float], eps: float = 1e-8) -> List[List[float]]:
    """Return the inverse of V used in SE(3) log."""
    angle: float = sqrt(theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2])
    K: List[List[float]] = _skew(theta)
    if angle < eps:
        angle_sq: float = angle * angle
        if angle_sq == 0.0:
            return _identity3()
        # 0.5 and 1/12 are the series coefficients for V^{-1}
        term1: float = 0.5
        term2: float = 1.0 / 12.0
        K2 = _matmul3(K, K)
        return _mat_add3(
            _identity3(),
            _mat_add3(_scale_mat3(K, -term1), _scale_mat3(K2, term2)),
        )
    # 0.5 converts full angle to half-angle for cot computation
    half: float = 0.5 * angle
    cot_half: float = cos(half) / sin(half)
    K2 = _matmul3(K, K)
    # The coefficient below is (1 - angle * cot(angle/2) / 2) / angle^2
    return _mat_add3(
        _identity3(),
        _mat_add3(
            _scale_mat3(K, -0.5),
            _scale_mat3(K2, (1.0 - angle * cot_half / 2.0) / (angle * angle)),
        ),
    )


def _identity3() -> List[List[float]]:
    """Return a 3x3 identity matrix."""
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _mat_add3(
    A: Sequence[Sequence[float]],
    B: Sequence[Sequence[float]],
) -> List[List[float]]:
    """Add two 3x3 matrices."""
    return [
        [A[0][0] + B[0][0], A[0][1] + B[0][1], A[0][2] + B[0][2]],
        [A[1][0] + B[1][0], A[1][1] + B[1][1], A[1][2] + B[1][2]],
        [A[2][0] + B[2][0], A[2][1] + B[2][1], A[2][2] + B[2][2]],
    ]


def _scale_mat3(A: Sequence[Sequence[float]], scale: float) -> List[List[float]]:
    """Scale a 3x3 matrix."""
    return [
        [A[0][0] * scale, A[0][1] * scale, A[0][2] * scale],
        [A[1][0] * scale, A[1][1] * scale, A[1][2] * scale],
        [A[2][0] * scale, A[2][1] * scale, A[2][2] * scale],
    ]
