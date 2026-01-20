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


class Quaternion:
    """Quaternion utilities for the AHRS core.

    Responsibility:
        Define the quaternion conventions used across the AHRS core and
        document the operations the math layer must provide.

    Purpose:
        Provide quaternion conventions and helper operations used by the AHRS
        core without tying the implementation to a specific math backend.

    Inputs/outputs:
        - Quaternions are stored as 4x1 vectors [w, x, y, z].
        - Rotation matrices are 3x3 with R(q_WB) mapping {W} -> {B}.
        - Angular rates are 3x1 vectors in rad/s.

    Dependencies:
        - Used by process, IMU, and magnetometer models.
        - Used by state mapping and EKF steps for attitude propagation.

    Public API (to be implemented):
        - normalize(q)
        - multiply(q_left, q_right)
        - to_matrix(q)
        - from_matrix(R)
        - omega_matrix(omega)
        - integrate(q, omega, dt)
        - small_angle_quat(delta_theta)
        - conjugate(q)

    Data contract:
        - q is a length-4 vector [w, x, y, z].
        - R is a 3x3 rotation matrix.
        - omega and delta_theta are length-3 vectors.

    Frames and units:
        - q_WB rotates world vectors into the body: v_B = R(q_WB) * v_W.
        - omega is body angular rate in {B} with units rad/s.

    Determinism and edge cases:
        - Pure math utilities with deterministic outputs for identical
          inputs.
        - Normalization must be well-defined and avoid data-dependent
          branching.
        - normalize() must handle near-zero norm with a deterministic fallback
          (for example, identity quaternion) and should record diagnostics.
        - After normalize(), from_matrix(), or integrate(), enforce a
          canonical sign with w >= 0. If w < 0, multiply q by -1 so q and
          -q map to the same rotation but yield deterministic outputs.
        - small_angle_quat() must be valid for ||delta_theta|| << 1 rad.

    Equations:
        Composition uses the Hamilton product with the convention:
            q_AB = q_CB ⊗ q_AC
            R(q_AB) = R(q_CB) * R(q_AC)

        Angular-rate matrix:
            Ω(ω) = [ 0   -ωx  -ωy  -ωz
                     ωx   0    ωz  -ωy
                     ωy  -ωz   0    ωx
                     ωz   ωy  -ωx   0 ]

            q̇ = 0.5 * Ω(ω) * q

    Numerical stability notes:
        - Always re-normalize after integration or composition.
        - Avoid explicit matrix inversion when possible.

    Suggested unit tests:
        - Identity quaternion yields identity rotation matrix.
        - Composition matches matrix multiplication order.
        - Small-angle integration matches first-order approximation.
        - Normalization handles very small norms deterministically.
    """

    @staticmethod
    def normalize(q: Sequence[float]) -> List[float]:
        Quaternion._validate_quat(q, "normalize")
        norm_sq: float = sum(value * value for value in q)
        eps: float = 1e-12
        if norm_sq < eps:
            return [1.0, 0.0, 0.0, 0.0]
        inv_norm: float = 1.0 / math.sqrt(norm_sq)
        result: List[float] = [value * inv_norm for value in q]
        if result[0] < 0.0:
            result = [-value for value in result]
        return result

    @staticmethod
    def conjugate(q: Sequence[float]) -> List[float]:
        Quaternion._validate_quat(q, "conjugate")
        return [q[0], -q[1], -q[2], -q[3]]

    @staticmethod
    def multiply(q_left: Sequence[float], q_right: Sequence[float]) -> List[float]:
        Quaternion._validate_quat(q_left, "multiply")
        Quaternion._validate_quat(q_right, "multiply")
        w1: float = q_left[0]
        x1: float = q_left[1]
        y1: float = q_left[2]
        z1: float = q_left[3]
        w2: float = q_right[0]
        x2: float = q_right[1]
        y2: float = q_right[2]
        z2: float = q_right[3]
        w: float = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x: float = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y: float = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z: float = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        result: List[float] = [w, x, y, z]
        return Quaternion.normalize(result)

    @staticmethod
    def to_matrix(q: Sequence[float]) -> List[List[float]]:
        qn: List[float] = Quaternion.normalize(q)
        w: float = qn[0]
        x: float = qn[1]
        y: float = qn[2]
        z: float = qn[3]
        xx: float = x * x
        yy: float = y * y
        zz: float = z * z
        wx: float = w * x
        wy: float = w * y
        wz: float = w * z
        xy: float = x * y
        xz: float = x * z
        yz: float = y * z
        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    @staticmethod
    def from_matrix(matrix: Sequence[Sequence[float]]) -> List[float]:
        Quaternion._validate_matrix(matrix, "from_matrix")
        m00: float = matrix[0][0]
        m11: float = matrix[1][1]
        m22: float = matrix[2][2]
        trace: float = m00 + m11 + m22
        w: float
        x: float
        y: float
        z: float
        s: float
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (matrix[2][1] - matrix[1][2]) / s
            y = (matrix[0][2] - matrix[2][0]) / s
            z = (matrix[1][0] - matrix[0][1]) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (matrix[2][1] - matrix[1][2]) / s
            x = 0.25 * s
            y = (matrix[0][1] + matrix[1][0]) / s
            z = (matrix[0][2] + matrix[2][0]) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (matrix[0][2] - matrix[2][0]) / s
            x = (matrix[0][1] + matrix[1][0]) / s
            y = 0.25 * s
            z = (matrix[1][2] + matrix[2][1]) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (matrix[1][0] - matrix[0][1]) / s
            x = (matrix[0][2] + matrix[2][0]) / s
            y = (matrix[1][2] + matrix[2][1]) / s
            z = 0.25 * s
        return Quaternion.normalize([w, x, y, z])

    @staticmethod
    def omega_matrix(omega: Sequence[float]) -> List[List[float]]:
        Quaternion._validate_vector(omega, 3, "omega_matrix")
        wx: float = omega[0]
        wy: float = omega[1]
        wz: float = omega[2]
        return [
            [0.0, -wx, -wy, -wz],
            [wx, 0.0, wz, -wy],
            [wy, -wz, 0.0, wx],
            [wz, wy, -wx, 0.0],
        ]

    @staticmethod
    def small_angle_quat(delta_theta: Sequence[float]) -> List[float]:
        Quaternion._validate_vector(delta_theta, 3, "small_angle_quat")
        half: float = 0.5
        q: List[float] = [1.0, half * delta_theta[0], half * delta_theta[1], half * delta_theta[2]]
        return Quaternion.normalize(q)

    @staticmethod
    def integrate(
        q: Sequence[float],
        omega: Sequence[float],
        dt: float,
    ) -> List[float]:
        Quaternion._validate_quat(q, "integrate")
        Quaternion._validate_vector(omega, 3, "integrate")
        delta_theta: List[float] = [omega[i] * dt for i in range(3)]
        angle: float = math.sqrt(sum(value * value for value in delta_theta))
        eps: float = 1e-6
        if angle < eps:
            dq: List[float] = Quaternion.small_angle_quat(delta_theta)
        else:
            axis_scale: float = 1.0 / angle
            axis: List[float] = [delta_theta[i] * axis_scale for i in range(3)]
            half_angle: float = 0.5 * angle
            sin_half: float = math.sin(half_angle)
            dq = [
                math.cos(half_angle),
                axis[0] * sin_half,
                axis[1] * sin_half,
                axis[2] * sin_half,
            ]
        return Quaternion.normalize(Quaternion.multiply(dq, q))

    @staticmethod
    def _validate_quat(q: Sequence[float], name: str) -> None:
        Quaternion._validate_vector(q, 4, name)

    @staticmethod
    def _validate_vector(vec: Sequence[float], size: int, name: str) -> None:
        if len(vec) != size:
            raise ValueError(f"{name} expects length-{size} vector")

    @staticmethod
    def _validate_matrix(matrix: Sequence[Sequence[float]], name: str) -> None:
        if len(matrix) != 3:
            raise ValueError(f"{name} expects 3x3 matrix")
        for row in matrix:
            if len(row) != 3:
                raise ValueError(f"{name} expects 3x3 matrix")
