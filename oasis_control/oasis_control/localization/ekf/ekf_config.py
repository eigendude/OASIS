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
        t_buffer_sec: Buffer span in seconds used for fixed-lag replay
        epsilon_wall_future: Max future offset in seconds allowed by wall clock
        dt_clock_jump_max: Threshold in seconds for detecting clock jumps
        dt_imu_max: Max IMU delta in seconds before skipping propagation
        pos_var: Initial position variance in m^2
        vel_var: Initial velocity variance in (m/s)^2
        ang_var: Initial angle variance in rad^2
        accel_noise_var: Process accel variance in (m/s^2)^2
        gyro_noise_var: Process gyro variance in (rad/s)^2
        gravity_mps2: Gravity magnitude in m/s^2
        max_dt_sec: Max integration step in seconds
        checkpoint_interval_sec: Replay checkpoint interval in seconds
        apriltag_pos_var: AprilTag position variance in m^2
        apriltag_yaw_var: AprilTag yaw variance in rad^2
        apriltag_gate_d2: AprilTag Mahalanobis gate threshold d^2
        tag_size_m: AprilTag edge length in meters
        tag_anchor_family: AprilTag family name for world anchor
        tag_anchor_id: AprilTag identifier for world anchor
        tag_landmark_prior_sigma_t_m: Tag translation prior sigma in meters
        tag_landmark_prior_sigma_rot_rad: Tag rotation prior sigma in radians
    """

    world_frame_id: str
    odom_frame_id: str
    body_frame_id: str

    t_buffer_sec: float
    epsilon_wall_future: float
    dt_clock_jump_max: float
    dt_imu_max: float

    pos_var: float
    vel_var: float
    ang_var: float
    accel_noise_var: float
    gyro_noise_var: float
    gravity_mps2: float
    max_dt_sec: float
    checkpoint_interval_sec: float
    apriltag_pos_var: float
    apriltag_yaw_var: float
    apriltag_gate_d2: float

    tag_size_m: float
    tag_anchor_family: str
    tag_anchor_id: int
    tag_landmark_prior_sigma_t_m: float
    tag_landmark_prior_sigma_rot_rad: float
