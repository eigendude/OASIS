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
# Full AHRS filter state snapshot at the current frontier time t_filter.
#
# This message exists so consumers can inspect ALL in-state parameters and
# their FULL (non-diagonalized) covariance in a single place.
#
# Conventions
# - header.stamp = t_filter (frontier)
# - header.frame_id = world frame ("world") for all world-expressed vectors
#
# Frames
# - {W}: world
# - {B}: base_link (body)
# - {I}: IMU frame
# - {M}: magnetometer frame
#

# Frontier timestamp and world frame_id
std_msgs/Header header

#
# Frame ids in use by the node (published contract)
#
string world_frame_id
string odom_frame_id
string body_frame_id

#
# Monotonic sequence index for state publications
#
uint64 state_seq

#
# True when the filter has consumed its initial priors and is running
#
bool initialized

#
# Navigation state
#

# Position of B expressed in W (m)
geometry_msgs/Point position_wb_m

# Velocity of B expressed in W (m/s)
geometry_msgs/Vector3 velocity_wb_mps

# Attitude (world -> body), unit quaternion
geometry_msgs/Quaternion orientation_wb

#
# IMU systematic parameters (in-state)
#

# Gyroscope bias b_g (rad/s), expressed in IMU frame {I} unless otherwise specified by the node
geometry_msgs/Vector3 gyro_bias

# Accelerometer bias b_a (m/s^2), expressed in IMU frame {I} unless otherwise specified by the node
geometry_msgs/Vector3 accel_bias

# Accelerometer scale/misalignment matrix A_a
# Units: unitless
# Layout: 3x3 row-major
float64[9] accel_a

#
# Extrinsics (sensor -> body)
#

# IMU -> body transform T_BI
geometry_msgs/Pose t_bi

# Magnetometer -> body transform T_BM
geometry_msgs/Pose t_bm

#
# Environment / reference vectors (in-state)
#

# Gravity vector expressed in world (m/s^2)
geometry_msgs/Vector3 gravity_w_mps2

# Earth magnetic field vector expressed in world (tesla)
geometry_msgs/Vector3 magnetic_field_w_t

#
# FULL covariance over the AHRS internal error-state.
#
# This is the single source of truth for "not diagonalized" covariance reporting.
#
# Matrix is row-major with dimensions error_dim x error_dim.
#
# IMPORTANT: error_state_names defines the exact ordering of this covariance.
#

# Error-state dimension (must match p.rows == p.cols == error_dim)
uint32 error_dim

# Names of each scalar error-state element, length = error_dim.
# Example entries:
# - "dp.x", "dp.y", "dp.z"
# - "dv.x", "dv.y", "dv.z"
# - "dtheta.x", "dtheta.y", "dtheta.z"
# - "dbg.x", ...
# - "dba.x", ...
# - "dAa.0", ..., "dAa.8"
# - "dT_BI.rho.x", ..., "dT_BI.theta.z"
# - "dT_BM.rho.x", ..., "dT_BM.theta.z"
# - "dg.x", ...
# - "dm.x", ...
string[] error_state_names

# Full error-state covariance P
oasis_msgs/Matrix p
