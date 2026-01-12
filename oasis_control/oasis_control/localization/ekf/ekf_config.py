################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

"""
Configuration data for EKF localization
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class EkfConfig:
    """
    Shared EKF configuration values

    Fields:
        world_frame_id: Global drift-corrected frame name
        odom_frame_id: Local continuous frame name
        body_frame_id: Body frame name for state output
        t_buffer_ns: Buffer span in nanoseconds used for fixed-lag replay
        epsilon_wall_future_ns: Max future offset in ns allowed by wall clock
        dt_clock_jump_max_ns: Threshold in ns for detecting clock jumps
        dt_imu_max_ns: Max IMU delta in ns before skipping propagation
        pos_var: Initial position variance in m^2
        vel_var: Initial velocity variance in (m/s)^2
        ang_var: Initial angle variance in rad^2
        accel_noise_var: Accel noise spectral density in (m/s^2)^2 / Hz
        gyro_noise_var: Gyro noise spectral density in (rad/s)^2 / Hz
        gravity_mps2: Gravity magnitude in m/s^2
        max_dt_ns: Max integration step in nanoseconds
        checkpoint_interval_ns: Replay checkpoint interval in nanoseconds
        apriltag_pos_var: AprilTag position variance in m^2
        apriltag_yaw_var: AprilTag yaw variance in rad^2
        apriltag_gate_d2: AprilTag Mahalanobis gate threshold d^2
        apriltag_reproj_rms_gate_px: AprilTag reprojection RMS gate, pixels
        tag_size_m: AprilTag edge length in meters
        tag_anchor_family: AprilTag family name for world anchor
        tag_anchor_id: AprilTag identifier for world anchor
        tag_landmark_prior_sigma_t_m: Tag translation prior sigma in meters
        tag_landmark_prior_sigma_rot_rad: Tag rotation prior sigma in radians
        extrinsic_prior_sigma_t_m: Extrinsic translation prior sigma in meters
        extrinsic_prior_sigma_rot_rad: Extrinsic rotation prior sigma in radians
        mag_alpha: Magnetometer covariance matching blend factor
        mag_r_min: Minimum magnetometer covariance matrix in tesla^2, row-major
        mag_r_max: Maximum magnetometer covariance matrix in tesla^2, row-major
        mag_r0_default: Default magnetometer covariance prior in tesla^2, row-major
        mag_world_t: Expected magnetic field vector in world frame, tesla
    """

    world_frame_id: str
    odom_frame_id: str
    body_frame_id: str

    t_buffer_ns: int
    epsilon_wall_future_ns: int
    dt_clock_jump_max_ns: int
    dt_imu_max_ns: int

    pos_var: float
    vel_var: float
    ang_var: float
    accel_noise_var: float
    gyro_noise_var: float
    gravity_mps2: float
    max_dt_ns: int
    checkpoint_interval_ns: int
    apriltag_pos_var: float
    apriltag_yaw_var: float
    apriltag_gate_d2: float
    apriltag_reproj_rms_gate_px: float

    tag_size_m: float
    tag_anchor_family: str
    tag_anchor_id: int
    tag_landmark_prior_sigma_t_m: float
    tag_landmark_prior_sigma_rot_rad: float
    extrinsic_prior_sigma_t_m: float
    extrinsic_prior_sigma_rot_rad: float
    mag_alpha: float
    mag_r_min: list[float]
    mag_r_max: list[float]
    mag_r0_default: list[float]
    mag_world_t: list[float]
