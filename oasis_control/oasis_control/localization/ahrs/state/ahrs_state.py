################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

class AhrsState:
    """Mean-state container for the AHRS core.

    Purpose:
        Represent the nominal state that is propagated by the process model
        and corrected by measurement updates.

    Public API (to be implemented):
        - copy()
        - apply_error(delta_x)
        - as_vector()
        - from_vector(x)
        - reset()

    Data contract:
        Required fields and shapes:
        - p_WB: position of body in world, shape (3,).
        - v_WB: velocity of body in world, shape (3,).
        - q_WB: unit quaternion [w, x, y, z].
        - omega_WB: Body angular rate: `ω_WB ∈ ℝ³` in `{B}` (rad/s), the angular
          velocity of `{B}` relative to `{W}` expressed in `{B}`.
        - b_g: gyro bias in {I}, shape (3,).
        - b_a: accel bias in {I}, shape (3,).
        - A_a: accel calibration matrix, shape (3, 3).
        - T_BI: IMU extrinsic transform (SE(3)).
        - T_BM: magnetometer extrinsic transform (SE(3)).
        - g_W: gravity vector in world, shape (3,).
        - m_W: magnetic field in world, shape (3,).

    Frames and units:
        - p_WB in meters, v_WB in meters per second.
        - q_WB rotates {W} -> {B}.
        - b_g in rad/s in {I}, b_a in m/s^2 in {I}.
        - A_a is unitless scale/misalignment.
        - T_BI, T_BM translations in meters, rotations unitless.
        - g_W in m/s^2, m_W in tesla.

    Responsibility:
        Define the nominal (mean) AHRS state elements and their ordering,
        units, and frames without any ROS types.

    Inputs/outputs:
        - Inputs are plain vectors/matrices representing state components.
        - Outputs are the same components, typically passed to process or
          measurement models.

    Dependencies:
        - Used by process_model, imu_model, mag_model, and filter steps.
        - Coupled with error_state and state_mapping for covariance
          operations.

    Determinism:
        State storage is deterministic; no hidden time or parameter lookups.

    Determinism and edge cases:
        - apply_error must use the canonical error-state ordering from
          StateMapping.
        - Quaternion normalization is required after applying delta_theta.

    Equations:
        - q_WB is updated with a small-angle quaternion: q <- dq ⊗ q.
        - Extrinsics use SE(3) perturbations: T <- Exp(delta_xi) * T.

    Numerical stability notes:
        - Normalize q_WB after any update.
        - Keep A_a invertible; enforce conditioning elsewhere.

    Suggested unit tests:
        - apply_error respects StateMapping index ranges.
        - Quaternion normalization maintains unit norm.
    """

    pass
