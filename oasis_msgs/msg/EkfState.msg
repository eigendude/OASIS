################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Full EKF state snapshot for runtime debugging and replay inspection
#

# Snapshot time and primary state frame
#
# header.frame_id is expected to be world
#
std_msgs/Header header

# World-frame nominal position p_WB
#
# Units: m
#
geometry_msgs/Point position_world

# World-frame nominal velocity v_WB
#
# Units: m/s
#
geometry_msgs/Vector3 velocity_world

# World-frame nominal orientation q_WB
geometry_msgs/Quaternion orientation_world

# Derived local odom -> base_link output summary
#
# Expected frames:
# - odom.header.frame_id: odom
# - odom.child_frame_id: base_link
#
nav_msgs/Odometry odom

# Gyroscope bias state b_g in base coordinates
#
# Units: rad/s
#
geometry_msgs/Vector3 gyro_bias

# Accelerometer bias state b_a in base coordinates
#
# Units: m/s^2
#
geometry_msgs/Vector3 accel_bias

# Fixed mounting transform T_BI from imu_link into base_link
geometry_msgs/Transform t_bi

# Fixed camera extrinsic transform T_BC from camera_link into base_link
geometry_msgs/Transform t_bc

# Number of initialized AprilTag landmarks in the active EKF state
#
# Units: landmarks
#
uint32 landmark_count

# Full EKF covariance flattened row-major
#
# Length is covariance_dim * covariance_dim
#
float64[] covariance

# State covariance dimension for the flattened covariance block
#
# Units: states
#
uint32 covariance_dim

# True once camera intrinsics have been cached for AprilTag updates
bool has_camera_info

# True once the fixed mounting transform has been initialized
bool has_mounting

# True once the AprilTag landmark map has at least one initialized entry
bool has_map
