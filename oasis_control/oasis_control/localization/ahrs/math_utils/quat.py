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

from math import cos, sin, sqrt
from typing import List, Sequence


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
    def normalize(q: Sequence[float], eps: float = 1e-12) -> List[float]:
        """Normalize quaternion and enforce a canonical sign."""
        if len(q) != 4:
            raise ValueError("Quaternion must have length 4")
        norm_sq: float = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
        # eps is a small-norm threshold for deterministic fallback
        if norm_sq < eps * eps:
            return [1.0, 0.0, 0.0, 0.0]
        inv_norm: float = 1.0 / sqrt(norm_sq)
        qn: List[float] = [
            q[0] * inv_norm,
            q[1] * inv_norm,
            q[2] * inv_norm,
            q[3] * inv_norm,
        ]
        if qn[0] < 0.0:
            qn = [-qn[0], -qn[1], -qn[2], -qn[3]]
        return qn

    @staticmethod
    def conjugate(q: Sequence[float]) -> List[float]:
        """Return the conjugate quaternion."""
        if len(q) != 4:
            raise ValueError("Quaternion must have length 4")
        return [q[0], -q[1], -q[2], -q[3]]

    @staticmethod
    def multiply(q_left: Sequence[float], q_right: Sequence[float]) -> List[float]:
        """Return the Hamilton product q_left ⊗ q_right."""
        if len(q_left) != 4 or len(q_right) != 4:
            raise ValueError("Quaternion must have length 4")
        w1, x1, y1, z1 = q_left
        w2, x2, y2, z2 = q_right
        q: List[float] = [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
        return Quaternion.normalize(q)

    @staticmethod
    def to_matrix(q: Sequence[float]) -> List[List[float]]:
        """Convert quaternion to a 3x3 rotation matrix."""
        if len(q) != 4:
            raise ValueError("Quaternion must have length 4")
        w, x, y, z = q
        xx: float = x * x
        yy: float = y * y
        zz: float = z * z
        wx: float = w * x
        wy: float = w * y
        wz: float = w * z
        xy: float = x * y
        xz: float = x * z
        yz: float = y * z
        # The factors of 2 follow the standard quaternion rotation formula
        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    @staticmethod
    def from_matrix(R: Sequence[Sequence[float]]) -> List[float]:
        """Convert a rotation matrix to a quaternion with canonical sign."""
        if len(R) != 3 or any(len(row) != 3 for row in R):
            raise ValueError("Rotation matrix must be 3x3")
        trace: float = R[0][0] + R[1][1] + R[2][2]
        if trace > 0.0:
            # s = 4 * qw is derived from the trace formula
            s: float = sqrt(trace + 1.0) * 2.0
            w: float = 0.25 * s
            x: float = (R[2][1] - R[1][2]) / s
            y: float = (R[0][2] - R[2][0]) / s
            z: float = (R[1][0] - R[0][1]) / s
        else:
            if R[0][0] > R[1][1] and R[0][0] > R[2][2]:
                s = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
                w = (R[2][1] - R[1][2]) / s
                x = 0.25 * s
                y = (R[0][1] + R[1][0]) / s
                z = (R[0][2] + R[2][0]) / s
            elif R[1][1] > R[2][2]:
                s = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
                w = (R[0][2] - R[2][0]) / s
                x = (R[0][1] + R[1][0]) / s
                y = 0.25 * s
                z = (R[1][2] + R[2][1]) / s
            else:
                s = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
                w = (R[1][0] - R[0][1]) / s
                x = (R[0][2] + R[2][0]) / s
                y = (R[1][2] + R[2][1]) / s
                z = 0.25 * s
        return Quaternion.normalize([w, x, y, z])

    @staticmethod
    def omega_matrix(omega: Sequence[float]) -> List[List[float]]:
        """Return the 4x4 omega matrix used for quaternion kinematics."""
        if len(omega) != 3:
            raise ValueError("Omega must have length 3")
        wx, wy, wz = omega
        return [
            [0.0, -wx, -wy, -wz],
            [wx, 0.0, wz, -wy],
            [wy, -wz, 0.0, wx],
            [wz, wy, -wx, 0.0],
        ]

    @staticmethod
    def small_angle_quat(delta_theta: Sequence[float]) -> List[float]:
        """Return a small-angle quaternion approximation."""
        if len(delta_theta) != 3:
            raise ValueError("delta_theta must have length 3")
        # 0.5 scales the vector part for first-order small-angle mapping
        q: List[float] = [
            1.0,
            0.5 * delta_theta[0],
            0.5 * delta_theta[1],
            0.5 * delta_theta[2],
        ]
        return Quaternion.normalize(q)

    @staticmethod
    def integrate(
        q: Sequence[float],
        omega: Sequence[float],
        dt: float,
        eps: float = 1e-9,
    ) -> List[float]:
        """Integrate quaternion using body-rate omega over dt."""
        if len(q) != 4:
            raise ValueError("Quaternion must have length 4")
        if len(omega) != 3:
            raise ValueError("Omega must have length 3")
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        delta_theta: List[float] = [
            omega[0] * dt,
            omega[1] * dt,
            omega[2] * dt,
        ]
        angle: float = sqrt(
            delta_theta[0] * delta_theta[0]
            + delta_theta[1] * delta_theta[1]
            + delta_theta[2] * delta_theta[2]
        )
        # eps guards the small-angle branch for stability
        if angle < eps:
            dq: List[float] = Quaternion.small_angle_quat(delta_theta)
        else:
            # half is the half-angle used in axis-angle to quaternion mapping
            half: float = 0.5 * angle
            sin_half: float = sin(half)
            axis_scale: float = sin_half / angle
            dq = [
                cos(half),
                delta_theta[0] * axis_scale,
                delta_theta[1] * axis_scale,
                delta_theta[2] * axis_scale,
            ]
            dq = Quaternion.normalize(dq)
        q_next: List[float] = Quaternion.multiply(dq, q)
        return Quaternion.normalize(q_next)
