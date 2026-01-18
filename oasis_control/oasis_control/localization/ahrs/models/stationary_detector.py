################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class StationaryDetector:
    """Covariance-aware stationary detector for deterministic AHRS replay.

    Responsibility:
        Compute deterministic stationary decisions from buffered IMU windows
        using full covariance whitening metrics and SPD checks.

    Purpose:
        Provide a deterministic mapping from a window of IMU packets and the
        current state to a StationaryPacket with ZUPT/no-turn covariances.

    Inputs/outputs:
        - Inputs: window_imu_packets (sequence of ImuPacket), AhrsState.
        - Outputs: StationaryPacket with window bounds, decision flag, and
          full R_v/R_omega covariances.

    Dependencies:
        - Uses LinearAlgebra for SPD checks and clamping.
        - Uses ImuModel math for residual definitions.
        - Driven by AhrsConfig thresholds and baseline covariances.

    Public API (to be implemented):
        - detect(window_imu_packets, state)

    Data contract:
        - window_imu_packets are sorted deterministically by t_meas_ns.
        - Each packet supplies full R_omega and R_accel (3, 3) covariances.
        - R_v and R_omega outputs are full SPD matrices.

    Frames and units:
        - Gyro residual ν_ω in {I}, units rad/s.
        - Accel residual ν_a in {I}, units m/s^2.
        - Covariances use squared units and remain full.

    Determinism and edge cases:
        - Identical buffered IMU packets must yield identical decisions.
        - Window bounds are defined by integer-ns min/max timestamps.
        - Reject the window if any R_omega/R_accel is non-SPD.
        - Empty windows must return is_stationary False with metadata.
        - No epsilon or float-time comparisons are allowed.

    Equations:
        Gyro whitening per sample:
            ν_ω,i = z_ω,i - (R_IB * ω_WB + b_g)
            d2_ω,i = ν_ω,iᵀ R_ω^{-1} ν_ω,i

        Accel whitening per sample:
            f_B = R_WB * (0 - g_W)
            f_I = R_IB * f_B
            a_hat_I = A_a^{-1} * f_I + b_a
            ν_a,i = z_a,i - a_hat_I
            d2_a,i = ν_a,iᵀ R_a^{-1} ν_a,i

        Aggregate deterministically:
            D_ω = mean(d2_ω,i) or sum(d2_ω,i)
            D_a = mean(d2_a,i) or sum(d2_a,i)

        Stationary decision:
            is_stationary = (D_ω < τ_ω) AND (D_a < τ_a)
            plus optional deterministic delta-variance tests.

    Numerical stability notes:
        - Never diagonalize covariances.
        - Clamp R_v and R_omega with LinearAlgebra.clamp_spd.

    Suggested unit tests:
        - Deterministic ordering yields identical D_ω/D_a.
        - Reject window when any covariance is non-SPD.
    """

    pass
