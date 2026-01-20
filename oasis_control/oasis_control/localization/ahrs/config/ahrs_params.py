################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################


class AhrsParams:
    """Configuration parameter definitions for the AHRS core.

    Responsibility:
        Document the configuration parameters used by the AHRS core, grouped
        by functional area (buffering, process noise, adaptation, frames).

    Purpose:
        Provide a structured list of parameters with units and expected ranges
        for the AHRS core components.

    Inputs/outputs:
        - Inputs: parameter dictionaries or configuration objects.
        - Outputs: validated parameter values for AHRS modules.

    Dependencies:
        - Used by AhrsConfig, ProcessModel, and ReplayEngine.

    Public API (to be implemented):
        - defaults()
        - validate()
        - from_dict(params)

    Data contract:
        Buffer/replay parameters:
        - t_buffer_sec: buffer horizon in seconds.
        - ε_wall_future_ns: allowable future timestamp tolerance in
          nanoseconds.
        - Δt_clock_jump_max_ns: clock jump detection threshold in
          nanoseconds.
        - Δt_imu_max_ns: maximum IMU time gap allowed for replay coverage
          in nanoseconds.

        Process noise intensities:
        - sigma_w_v: accel smoothness prior (m/s^2 / sqrt(s)).
        - sigma_w_omega: angular accel prior (rad/s^2 / sqrt(s)).
        - sigma_w_bg: gyro bias random walk (rad/s / sqrt(s)).
        - sigma_w_ba: accel bias random walk (m/s^2 / sqrt(s)).
        - sigma_w_Aa: accel calibration random walk (1 / sqrt(s)).
        - sigma_w_BI: IMU extrinsics random walk placeholder.
          Temporary 6D noise uses diag([sigma_rho^2 I3,
          sigma_theta^2 I3]) with sigma_rho in m / sqrt(s) and
          sigma_theta in rad / sqrt(s).
        - sigma_w_BM: mag extrinsics random walk placeholder.
          Temporary 6D noise uses diag([sigma_rho^2 I3,
          sigma_theta^2 I3]) with sigma_rho in m / sqrt(s) and
          sigma_theta in rad / sqrt(s).
        - Preferred split parameters: sigma_w_BI_rho, sigma_w_BI_theta,
          sigma_w_BM_rho, sigma_w_BM_theta.
        - sigma_w_g: gravity random walk (m/s^2 / sqrt(s)).
        - sigma_w_m: magnetic field random walk (tesla / sqrt(s)).

        Magnetometer adaptation:
        - alpha: adaptation rate in [0, 1].
        - R_min: minimum SPD covariance (tesla^2).
        - R_max: maximum SPD covariance (tesla^2).
        - R_m0_policy: initial R_m policy for startup.

        Stationary detection and pseudo-measurements:
        - t_stationary_window_ns: stationary window length in nanoseconds.
        - tau_omega: gyro whitened threshold (dimensionless).
        - tau_accel: accel whitened threshold (dimensionless).
        - R_v0: baseline ZUPT covariance (3, 3), full SPD.
        - R_omega0: baseline no-turn covariance (3, 3), full SPD.

        Frame identifiers:
        - world_frame, odom_frame, base_frame, imu_frame, mag_frame.
        - Used only by higher-level integration; core is ROS-agnostic.

        Naming + conversion:
        - ns everywhere except buffer length.
        - The only seconds input is t_buffer_sec, which is converted once in
          AhrsConfig into t_buffer_ns using a deterministic rounding rule.
        - No float seconds are used for keying, ordering, equality,
          attachment, or replay.

    Frames and units:
        - Units match Units definitions and process noise conventions.

    Determinism and edge cases:
        - Parameters are explicit inputs; no implicit ROS parameter access.
        - validate() must reject invalid ranges (negative variances).
        - alpha outside [0, 1] should be rejected.

    Equations:
        - No equations; parameters configure other modules.

    Numerical stability notes:
        - Noise values too small may cause S to be ill-conditioned.

    Suggested unit tests:
        - validate rejects negative variances.
        - defaults produce a consistent parameter set.
    """

    pass
