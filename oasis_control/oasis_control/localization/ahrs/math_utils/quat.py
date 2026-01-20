################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Quaternion utilities for the AHRS core.

Responsibility:
    Define the quaternion conventions used across the AHRS core and document
    the operations the math layer must provide.

Inputs/outputs:
    - Quaternions are stored as 4x1 vectors [w, x, y, z].
    - Rotation matrices are 3x3 with R(q_WB) mapping {W} -> {B}.
    - Angular rates are 3x1 vectors in rad/s.

Dependencies:
    - Used by process, IMU, and magnetometer models.
    - Used by state mapping and EKF steps for attitude propagation.

Determinism:
    Pure math utilities with deterministic outputs for identical inputs.
    Normalization must be well-defined and avoid data-dependent branching.
"""


class Quaternion:
    """Quaternion math utilities for world-to-body rotations.

    Purpose:
        Provide quaternion conventions and helper operations used by the AHRS
        core without tying the implementation to a specific math backend.

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

    pass
