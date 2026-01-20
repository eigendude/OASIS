################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""Configuration parameter definitions for the AHRS core.

Responsibility:
    Document the configuration parameters used by the AHRS core, grouped by
    functional area (buffering, process noise, adaptation, frames).

Inputs/outputs:
    - Inputs: parameter dictionaries or configuration objects.
    - Outputs: validated parameter values for AHRS modules.

Dependencies:
    - Used by AhrsConfig, ProcessModel, and ReplayEngine.

Determinism:
    Parameters are explicit inputs; no implicit ROS parameter access.
"""


class AhrsParams:
    """Parameter schema for configuring the AHRS core.

    Purpose:
        Provide a structured list of parameters with units and expected ranges
        for the AHRS core components.

    Public API (to be implemented):
        - defaults()
        - validate()
        - from_dict(params)

    Data contract:
        Buffer/replay parameters:
        - T_buffer_sec: buffer horizon in seconds.
        - t_future_tol_sec: allowable future timestamp tolerance.
        - t_past_tol_sec: allowable past timestamp tolerance.

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

        Frame identifiers:
        - world_frame, odom_frame, base_frame, imu_frame, mag_frame.
        - Used only by higher-level integration; core is ROS-agnostic.

    Frames and units:
        - Units match Units definitions and process noise conventions.

    Determinism and edge cases:
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
