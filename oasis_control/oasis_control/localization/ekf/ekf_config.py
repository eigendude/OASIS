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

    tag_size_m: float
    tag_anchor_family: str
    tag_anchor_id: int
    tag_landmark_prior_sigma_t_m: float
    tag_landmark_prior_sigma_rot_rad: float
