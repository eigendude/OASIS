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
Configuration data for AHRS localization
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class AhrsConfig:
    """
    Shared AHRS configuration values

    Fields:
        world_frame_id: Global drift-corrected frame name
        odom_frame_id: Local continuous frame name
        body_frame_id: Body frame name for state output
        t_buffer_sec: Buffer span in seconds used for fixed-lag replay
        epsilon_wall_future_sec: Max future offset in seconds allowed by wall clock
        dt_clock_jump_max_sec: Threshold in seconds for detecting clock jumps
        dt_imu_max_sec: Max IMU delta in seconds before skipping propagation
        gyro_gate_d2_threshold: Mahalanobis gate threshold for gyro updates
        accel_gate_d2_threshold: Mahalanobis gate threshold for accel updates
        mag_alpha: Magnetometer covariance matching factor, unitless
        mag_r_min: Minimum magnetometer covariance matrix, row-major
        mag_r_max: Maximum magnetometer covariance matrix, row-major
        mag_r0: Default initial magnetometer covariance matrix, row-major
        q_v: Velocity process noise placeholder
        q_w: Angular velocity process noise placeholder
        q_bg: Gyro bias process noise placeholder
        q_ba: Accel bias process noise placeholder
        q_a: Accel scale process noise placeholder
        q_bi: IMU extrinsic process noise placeholder
        q_bm: Magnetometer extrinsic process noise placeholder
        q_g: Gravity process noise placeholder
        q_m: Magnetic field process noise placeholder
    """

    world_frame_id: str
    odom_frame_id: str
    body_frame_id: str

    t_buffer_sec: float
    epsilon_wall_future_sec: float
    dt_clock_jump_max_sec: float
    dt_imu_max_sec: float
    gyro_gate_d2_threshold: float
    accel_gate_d2_threshold: float

    mag_alpha: float
    mag_r_min: list[float]
    mag_r_max: list[float]
    mag_r0: list[float]

    q_v: float
    q_w: float
    q_bg: float
    q_ba: float
    q_a: float
    q_bi: float
    q_bm: float
    q_g: float
    q_m: float
