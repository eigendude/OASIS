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
# Calibration parameters and uncertainty for an IMU
#
# This message carries systematic (parameter) uncertainty separately from the
# per-sample noise covariance in sensor_msgs/Imu.
#
# Model
# - Accelerometer:
#     a_cal = A * (a_raw - b_a)
# - Gyroscope:
#     w_cal = w_raw - b_g
#

std_msgs/Header header

#
# True when calibration fields are populated and safe to apply.
#
bool valid

#
# Gravity magnitude assumed during calibration.
#
# Units: m/s^2
#
float64 gravity_mps2

#
# Number of samples used in the calibration fit (accelerometer).
#
# Units: samples
#
uint32 fit_sample_count

#
# Accelerometer bias b_a
#
# Units: m/s^2
#
geometry_msgs/Vector3 accel_bias

#
# Accelerometer scale/misalignment matrix A
#
# Units: unitless
# Layout: 3x3 row-major
#
float64[9] accel_a

#
# Covariance of accelerometer calibration parameters
#
# Units:
# - Bias terms: (m/s^2)^2
# - A terms:    unitless^2
#
# Params (full 12): [bx, by, bz, a00, a01, a02, a10, a11, a12, a20, a21, a22]
# Layout: 12x12 row-major
#
float64[144] accel_param_cov

#
# RMS gravity residual for fit quality
#
# Units: m/s^2
# Meaning: rms(|a_cal|-g) over fit samples
#
float64 rms_residual_mps2

#
# Gyroscope bias b_g
#
# Units: rad/s
#
geometry_msgs/Vector3 gyro_bias

#
# Covariance of gyroscope bias estimate Cov(b_g)
#
# Units: (rad/s)^2
# Layout: 3x3 row-major
#
float64[9] gyro_bias_cov

#
# Mean temperature of samples used for this estimate
#
# Units: deg C
#
float64 temperature_c

#
# Temperature variance of samples used for this estimate
#
# Units: (deg C)^2
#
float64 temperature_var_c2
