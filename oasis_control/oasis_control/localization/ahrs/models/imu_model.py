################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""IMU measurement models for the AHRS core.

Responsibility:
    Define the gyro and accelerometer measurement predictions and residuals
    used during EKF updates.

Inputs/outputs:
    - Inputs: AhrsState, ImuPacket (z_ω, z_a), and extrinsics T_BI.
    - Outputs: predicted measurements, residuals, and Jacobians.

Dependencies:
    - Uses Quaternion utilities and SE(3) extrinsics.
    - Consumed by UpdateStep for gyro/accel updates.

Determinism:
    Deterministic mapping from state to predicted IMU measurements.
"""


class ImuModel:
    """Gyro and accelerometer measurement models for the AHRS EKF.

    Purpose:
        Provide predicted IMU measurements and Jacobians for gyro and accel
        updates without treating IMU data as process inputs.

    Public API (to be implemented):
        - predict_gyro(state)
        - predict_accel(state)
        - residual_gyro(z_omega, z_hat)
        - residual_accel(z_accel, z_hat)
        - jacobian_gyro(state)
        - jacobian_accel(state)

    Data contract:
        - z_omega is a 3x1 gyro measurement in {I}.
        - z_accel is a 3x1 accel measurement in {I}.
        - R_omega and R_accel are 3x3 covariances.

    Frames and units:
        - Gyro residual ν is in {I} with units rad/s.
        - Accel residual ν is in {I} with units m/s^2.
        - Specific force convention: accel measures a - g, so at rest
          z_accel ≈ -g expressed in the sensor frame.

    Determinism and edge cases:
        - Residual sign convention is ν = z - z_hat.
        - Gyro update primarily touches omega_WB and b_g.
        - Accel update primarily touches v_dot, g_W, q_WB, b_a, A_a.
        - If A_a is near-singular, reject or condition the update.

    Equations:
        Gyro prediction in {I}:
            ω_hat_I = R_IB * ω_WB + b_g
            ν = z_ω - ω_hat_I

        Accel prediction in {I}:
            a_WB := 0 (deterministic mean of the process prior)
            f_B = R_WB * (a_WB - g_W)
            f_I = R_IB * f_B
            a_hat_I = A_a^{-1} * f_I + b_a
            ν = z_a - a_hat_I

        Notes:
            a_WB := 0 reflects the process model mean (smoothness prior with
            zero mean), supports gravity initialization, and avoids
            introducing a separate acceleration state. The process model uses
            v̇_WB = w_v for propagation. A future extension may introduce an
            explicit acceleration state or a deterministic finite-difference
            policy, but the current spec uses a_WB := 0.

    Numerical stability notes:
        - Use a stable inversion for A_a.
        - Keep quaternion normalized when computing R_WB.

    Suggested unit tests:
        - At rest with v̇=0, z_accel matches -g in sensor frame.
        - Residual sign convention matches ν = z - z_hat.
    """

    pass
