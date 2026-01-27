################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#
# AHRS Mounting Calibration result snapshot.
#
# Publishes the current mounting-rotation estimates (IMU->body and Mag->body),
# associated covariances, nuisance parameters, and status/quality metrics.
#
# Contract highlights:
# - Rotations only: translations are NOT estimated and are NOT included here.
# - Quaternions are explicitly stored in [w, x, y, z] order.
# - Rotation covariances are 3x3 tangent-space covariances (rad^2), row-major.
#

std_msgs/Header header

# -----------------------------
# Frames (stable identifiers)
# -----------------------------

# Body frame (parent frame for TF), e.g. "base_link"
string base_frame

# IMU sensor frame, from imu_raw.header.frame_id, e.g. "imu_link"
string imu_frame

# Magnetometer sensor frame, from magnetic_field.header.frame_id, e.g. "mag_link"
string mag_frame

# -----------------------------
# Flags / status
# -----------------------------

# True when the solution is anchored to base_link (reference pose captured)
bool anchored

# True when a valid magnetometer reference could not be established
bool mag_reference_invalid

# True when a magnetic disturbance is detected (or mag factors are being dropped)
bool mag_disturbance_detected

# True if the mag direction covariance prior was initialized from driver covariance
bool mag_dir_prior_from_driver_cov

# -----------------------------
# Mount rotations (primary outputs)
# -----------------------------

# Quaternion for R_BI (IMU {I} -> body {B}), order [w,x,y,z]
float64[4] q_bi_wxyz

# Quaternion for R_BM (Mag {M} -> body {B}), order [w,x,y,z]
float64[4] q_bm_wxyz

# 3x3 tangent-space covariance for R_BI (rad^2), row-major
float64[9] cov_r_bi_row_major_3x3

# 3x3 tangent-space covariance for R_BM (rad^2), row-major
float64[9] cov_r_bm_row_major_3x3

# -----------------------------
# Nuisance snapshot (refined online)
# -----------------------------

# Accel bias b_a (m/s^2), used in: a_corr = A_a (a_raw - b_a)
float64[3] accel_bias_mps2

# Accel scale/misalignment matrix A_a (row-major 3x3)
float64[9] accel_a_row_major_3x3

# Optional accel systematic covariance over [b_a (3), A_a (9)] => 12x12 row-major.
# Leave all zeros if not tracked.
float64[144] accel_param_cov_row_major_12x12

# Gyro bias b_g (rad/s), used in: omega_corr = omega_raw - b_g
float64[3] gyro_bias_rads

# Gyro bias covariance 3x3 (rad^2/s^2), row-major. Zeros if not tracked.
float64[9] gyro_bias_cov_row_major_3x3

# Optional magnetometer offset b_m (tesla) used in: m_corr = m_raw - b_m.
# Zeros if not estimated/used.
float64[3] mag_offset_t

# Optional mag offset covariance 3x3 (tesla^2), row-major. Zeros if not tracked.
float64[9] mag_offset_cov_row_major_3x3

# Current direction-residual noise model R_m (unitless^2 ~ rad^2), row-major 3x3.
# This is the bounded/adaptive covariance used for direction residuals.
float64[9] mag_r_m_unitless2_row_major_3x3

# Initial direction-residual noise model R_m0 (unitless^2 ~ rad^2), row-major 3x3.
float64[9] mag_r_m0_unitless2_row_major_3x3

# -----------------------------
# Quality / bookkeeping
# -----------------------------

# Total raw samples ingested (IMU pairs) since start/reset
uint32 raw_samples

# Number of steady segments emitted by gating
uint32 steady_segments

# Number of active keyframes in clustering
uint32 keyframes

# Direction-residual RMS for accel (unitless ~ radians)
float64 accel_residual_rms

# Direction-residual RMS for mag (unitless ~ radians). 0 when mag unused/invalid.
float64 mag_residual_rms

# Maximum angle between gravity directions across keyframes (deg)
float64 gravity_max_angle_deg

# Maximum angle between horizontal mag projections across keyframes (deg)
float64 mag_proj_max_angle_deg
