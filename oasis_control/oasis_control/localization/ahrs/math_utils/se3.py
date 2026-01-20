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
        t_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
        t_bc: Tuple[Sequence[Sequence[float]], Sequence[float]],
    ) -> Tuple[List[List[float]], List[float]]:
        r_ab, p_ab = t_ab
        r_bc, p_bc = t_bc
        Se3._validate_rotation(r_ab, "compose")
        Se3._validate_rotation(r_bc, "compose")
        Se3._validate_vector(p_ab, 3, "compose")
        Se3._validate_vector(p_bc, 3, "compose")
        r_ac: List[List[float]] = Se3._matmul3(r_ab, r_bc)
        p_ac: List[float] = Se3._matvec3(r_ab, p_bc)
        p_ac = [p_ac[i] + p_ab[i] for i in range(3)]
        return r_ac, p_ac

    @staticmethod
    def inverse(
        t_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
    ) -> Tuple[List[List[float]], List[float]]:
        r_ab, p_ab = t_ab
        Se3._validate_rotation(r_ab, "inverse")
        Se3._validate_vector(p_ab, 3, "inverse")
        r_ba: List[List[float]] = Se3._transpose3(r_ab)
        p_ba: List[float] = Se3._matvec3(r_ba, p_ab)
        p_ba = [-value for value in p_ba]
        return r_ba, p_ba

    @staticmethod
    def apply(
        t_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
        p_b: Sequence[float],
    ) -> List[float]:
        r_ab, p_ab = t_ab
        Se3._validate_rotation(r_ab, "apply")
        Se3._validate_vector(p_ab, 3, "apply")
        Se3._validate_vector(p_b, 3, "apply")
        p_a: List[float] = Se3._matvec3(r_ab, p_b)
        return [p_a[i] + p_ab[i] for i in range(3)]

    @staticmethod
    def to_matrix(
        t_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
    ) -> List[List[float]]:
        r_ab, p_ab = t_ab
        Se3._validate_rotation(r_ab, "to_matrix")
        Se3._validate_vector(p_ab, 3, "to_matrix")
        return [
            [r_ab[0][0], r_ab[0][1], r_ab[0][2], p_ab[0]],
            [r_ab[1][0], r_ab[1][1], r_ab[1][2], p_ab[1]],
            [r_ab[2][0], r_ab[2][1], r_ab[2][2], p_ab[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def from_matrix(
        matrix: Sequence[Sequence[float]],
    ) -> Tuple[List[List[float]], List[float]]:
        if len(matrix) != 4:
            raise ValueError("from_matrix expects 4x4 matrix")
        for row in matrix:
            if len(row) != 4:
                raise ValueError("from_matrix expects 4x4 matrix")
        last_row: Sequence[float] = matrix[3]
        if any(abs(last_row[i] - value) > 1e-9 for i, value in enumerate([0.0, 0.0, 0.0, 1.0])):
            raise ValueError("from_matrix expects homogeneous last row")
        r_ab: List[List[float]] = [list(matrix[i][:3]) for i in range(3)]
        p_ab: List[float] = [matrix[i][3] for i in range(3)]
        Se3._validate_rotation(r_ab, "from_matrix")
        return r_ab, p_ab

    @staticmethod
    def hat(delta_xi: Sequence[float]) -> List[List[float]]:
        Se3._validate_vector(delta_xi, 6, "hat")
        v: Sequence[float] = delta_xi[:3]
        omega: Sequence[float] = delta_xi[3:]
        skew: List[List[float]] = Se3._skew(omega)
        return [
            [skew[0][0], skew[0][1], skew[0][2], v[0]],
            [skew[1][0], skew[1][1], skew[1][2], v[1]],
            [skew[2][0], skew[2][1], skew[2][2], v[2]],
            [0.0, 0.0, 0.0, 0.0],
        ]

    @staticmethod
    def vee(xi_hat: Sequence[Sequence[float]]) -> List[float]:
        if len(xi_hat) != 4:
            raise ValueError("vee expects 4x4 matrix")
        for row in xi_hat:
            if len(row) != 4:
                raise ValueError("vee expects 4x4 matrix")
        v: List[float] = [xi_hat[0][3], xi_hat[1][3], xi_hat[2][3]]
        omega: List[float] = Se3._unskew([list(row[:3]) for row in xi_hat[:3]])
        return [v[0], v[1], v[2], omega[0], omega[1], omega[2]]

    @staticmethod
    def exp(delta_xi: Sequence[float]) -> Tuple[List[List[float]], List[float]]:
        Se3._validate_vector(delta_xi, 6, "exp")
        delta_rho: List[float] = list(delta_xi[:3])
        delta_theta: List[float] = list(delta_xi[3:])
        r: List[List[float]] = Se3._so3_exp(delta_theta)
        v_mat: List[List[float]] = Se3._so3_v_matrix(delta_theta)
        p: List[float] = Se3._matvec3(v_mat, delta_rho)
        return r, p

    @staticmethod
    def log(
        t_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
    ) -> List[float]:
        r_ab, p_ab = t_ab
        Se3._validate_rotation(r_ab, "log")
        Se3._validate_vector(p_ab, 3, "log")
        delta_theta: List[float] = Se3._so3_log(r_ab)
        v_inv: List[List[float]] = Se3._so3_v_inverse(delta_theta)
        delta_rho: List[float] = Se3._matvec3(v_inv, p_ab)
        return [
            delta_rho[0],
            delta_rho[1],
            delta_rho[2],
            delta_theta[0],
            delta_theta[1],
            delta_theta[2],
        ]

    @staticmethod
    def adjoint(
        t_ab: Tuple[Sequence[Sequence[float]], Sequence[float]],
    ) -> List[List[float]]:
        r_ab, p_ab = t_ab
        Se3._validate_rotation(r_ab, "adjoint")
        Se3._validate_vector(p_ab, 3, "adjoint")
        p_skew: List[List[float]] = Se3._skew(p_ab)
        pr: List[List[float]] = Se3._matmul3(p_skew, r_ab)
        top: List[List[float]] = [
            r_ab[0] + pr[0],
            r_ab[1] + pr[1],
            r_ab[2] + pr[2],
        ]
        bottom: List[List[float]] = [
            [0.0, 0.0, 0.0, r_ab[0][0], r_ab[0][1], r_ab[0][2]],
            [0.0, 0.0, 0.0, r_ab[1][0], r_ab[1][1], r_ab[1][2]],
            [0.0, 0.0, 0.0, r_ab[2][0], r_ab[2][1], r_ab[2][2]],
        ]
        return [
            top[0],
            top[1],
            top[2],
            bottom[0],
            bottom[1],
            bottom[2],
        ]

    @staticmethod
    def _skew(vec: Sequence[float]) -> List[List[float]]:
        Se3._validate_vector(vec, 3, "skew")
        x, y, z = vec
        return [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]

    @staticmethod
    def _unskew(matrix: Sequence[Sequence[float]]) -> List[float]:
        Se3._validate_rotation(matrix, "unskew")
        return [matrix[2][1], matrix[0][2], matrix[1][0]]

    @staticmethod
    def _so3_exp(delta_theta: Sequence[float]) -> List[List[float]]:
        Se3._validate_vector(delta_theta, 3, "so3_exp")
        angle: float = math.sqrt(sum(value * value for value in delta_theta))
        if angle < 1e-8:
            skew: List[List[float]] = Se3._skew(delta_theta)
            skew2: List[List[float]] = Se3._matmul3(skew, skew)
            return Se3._matadd3(Se3._identity3(), skew, 0.5, skew2, 1.0 / 6.0)
        axis: List[float] = [value / angle for value in delta_theta]
        skew: List[List[float]] = Se3._skew(axis)
        skew2: List[List[float]] = Se3._matmul3(skew, skew)
        sin_term: float = math.sin(angle)
        cos_term: float = math.cos(angle)
        term1: List[List[float]] = Se3._mat_scale(skew, sin_term)
        term2: List[List[float]] = Se3._mat_scale(skew2, 1.0 - cos_term)
        return Se3._matadd3(Se3._identity3(), term1, 1.0, term2, 1.0)

    @staticmethod
    def _so3_log(matrix: Sequence[Sequence[float]]) -> List[float]:
        Se3._validate_rotation(matrix, "so3_log")
        trace: float = matrix[0][0] + matrix[1][1] + matrix[2][2]
        cos_angle: float = 0.5 * (trace - 1.0)
        cos_angle = min(1.0, max(-1.0, cos_angle))
        angle: float = math.acos(cos_angle)
        if angle < 1e-8:
            return [
                0.5 * (matrix[2][1] - matrix[1][2]),
                0.5 * (matrix[0][2] - matrix[2][0]),
                0.5 * (matrix[1][0] - matrix[0][1]),
            ]
        if math.pi - angle < 1e-6:
            diag: List[float] = [matrix[0][0], matrix[1][1], matrix[2][2]]
            axis: List[float] = [0.0, 0.0, 0.0]
            idx: int = diag.index(max(diag))
            axis[idx] = math.sqrt(max(0.0, (diag[idx] + 1.0) * 0.5))
            if idx == 0:
                axis[1] = matrix[0][1] / (2.0 * axis[0])
                axis[2] = matrix[0][2] / (2.0 * axis[0])
            elif idx == 1:
                axis[0] = matrix[0][1] / (2.0 * axis[1])
                axis[2] = matrix[1][2] / (2.0 * axis[1])
            else:
                axis[0] = matrix[0][2] / (2.0 * axis[2])
                axis[1] = matrix[1][2] / (2.0 * axis[2])
            return [axis[0] * angle, axis[1] * angle, axis[2] * angle]
        scale: float = angle / (2.0 * math.sin(angle))
        return [
            scale * (matrix[2][1] - matrix[1][2]),
            scale * (matrix[0][2] - matrix[2][0]),
            scale * (matrix[1][0] - matrix[0][1]),
        ]

    @staticmethod
    def _so3_v_matrix(delta_theta: Sequence[float]) -> List[List[float]]:
        Se3._validate_vector(delta_theta, 3, "so3_v_matrix")
        angle: float = math.sqrt(sum(value * value for value in delta_theta))
        skew: List[List[float]] = Se3._skew(delta_theta)
        skew2: List[List[float]] = Se3._matmul3(skew, skew)
        if angle < 1e-8:
            return Se3._matadd3(Se3._identity3(), skew, 0.5, skew2, 1.0 / 6.0)
        angle2: float = angle * angle
        sin_term: float = math.sin(angle)
        cos_term: float = math.cos(angle)
        coeff1: float = (1.0 - cos_term) / angle2
        coeff2: float = (angle - sin_term) / (angle2 * angle)
        term1: List[List[float]] = Se3._mat_scale(skew, coeff1)
        term2: List[List[float]] = Se3._mat_scale(skew2, coeff2)
        return Se3._matadd3(Se3._identity3(), term1, 1.0, term2, 1.0)

    @staticmethod
    def _so3_v_inverse(delta_theta: Sequence[float]) -> List[List[float]]:
        Se3._validate_vector(delta_theta, 3, "so3_v_inverse")
        angle: float = math.sqrt(sum(value * value for value in delta_theta))
        skew: List[List[float]] = Se3._skew(delta_theta)
        skew2: List[List[float]] = Se3._matmul3(skew, skew)
        if angle < 1e-8:
            return Se3._matadd3(Se3._identity3(), skew, -0.5, skew2, 1.0 / 12.0)
        angle2: float = angle * angle
        sin_term: float = math.sin(angle)
        cos_term: float = math.cos(angle)
        denom: float = 2.0 * (1.0 - cos_term)
        coeff: float = 1.0 - (angle * sin_term) / denom
        coeff2: float = coeff / angle2
        term1: List[List[float]] = Se3._mat_scale(skew, -0.5)
        term2: List[List[float]] = Se3._mat_scale(skew2, coeff2)
        return Se3._matadd3(Se3._identity3(), term1, 1.0, term2, 1.0)

    @staticmethod
    def _matmul3(
        left: Sequence[Sequence[float]],
        right: Sequence[Sequence[float]],
    ) -> List[List[float]]:
        return [
            [
                left[i][0] * right[0][j]
                + left[i][1] * right[1][j]
                + left[i][2] * right[2][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

    @staticmethod
    def _matvec3(matrix: Sequence[Sequence[float]], vec: Sequence[float]) -> List[float]:
        return [
            matrix[0][0] * vec[0] + matrix[0][1] * vec[1] + matrix[0][2] * vec[2],
            matrix[1][0] * vec[0] + matrix[1][1] * vec[1] + matrix[1][2] * vec[2],
            matrix[2][0] * vec[0] + matrix[2][1] * vec[1] + matrix[2][2] * vec[2],
        ]

    @staticmethod
    def _transpose3(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
        return [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ]

    @staticmethod
    def _mat_scale(matrix: Sequence[Sequence[float]], scale: float) -> List[List[float]]:
        return [[scale * value for value in row] for row in matrix]

    @staticmethod
    def _matadd3(
        base: Sequence[Sequence[float]],
        mat_a: Sequence[Sequence[float]],
        scale_a: float,
        mat_b: Sequence[Sequence[float]],
        scale_b: float,
    ) -> List[List[float]]:
        return [
            [
                base[i][j] + scale_a * mat_a[i][j] + scale_b * mat_b[i][j]
                for j in range(3)
            ]
            for i in range(3)
        ]

    @staticmethod
    def _identity3() -> List[List[float]]:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    @staticmethod
    def _validate_vector(vec: Sequence[float], size: int, name: str) -> None:
        if len(vec) != size:
            raise ValueError(f"{name} expects length-{size} vector")

    @staticmethod
    def _validate_rotation(matrix: Sequence[Sequence[float]], name: str) -> None:
        if len(matrix) != 3:
            raise ValueError(f"{name} expects 3x3 matrix")
        for row in matrix:
            if len(row) != 3:
                raise ValueError(f"{name} expects 3x3 matrix")
