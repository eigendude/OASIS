################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Vibration / motion-quality metrics derived from an IMU stream.
#
# Intended for gating (bias learning / EKF accel updates), diagnostics, and track health.
#

# Stamp = end time of the window; frame_id = IMU frame
std_msgs/Header header

# Rolling window length used for the stats (s)
float32 window_sec

# Effective sample rate observed over the window (Hz)
float32 sample_rate_hz

#
# Fast-motion / vibration energy (gravity suppressed via high-pass, differencing, or jerk)
#

# RMS of high-passed acceleration per-axis (m/s^2)
geometry_msgs/Vector3 accel_hp_rms_mps2

# RMS magnitude of high-passed acceleration (m/s^2)
float32 accel_hp_rms_mag_mps2

# RMS jerk magnitude over the window (m/s^3)
float32 jerk_rms_mps3

# Variance of acceleration magnitude |a| over the window (m^2/s^4)
float32 accel_mag_var_mps4

#
# Rotation activity
#

# RMS gyro magnitude over the window (rad/s)
float32 gyro_rms_rads

#
# Composite quality / gating signals
#

# Unitless motion/noise score (higher = more dynamic / noisy)
float32 motion_score

# Suggested weight for learning/measurement updates in [0..1]
float32 confidence

# True if the window appears stationary (heuristic)
bool stationary

# True if the most recent sample was fresh (DRDY-based) when available
bool data_ready

#
# Sanity-check helpers
#

# Low-passed acceleration magnitude expressed in g (unitless)
float32 accel_g_lp
