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
# Structured diagnostics for the AHRS mounting calibration node.
#
# This is a typed alternative to stuffing everything into DiagnosticArray.
# It’s intended for logging, dashboards, and test assertions.
#

std_msgs/Header header

# -----------------------------
# Timing / drops
# -----------------------------

# Count of imu_raw messages dropped because no ExactTime imu_calibration matched
uint32 imu_raw_dropped_no_calibration

# Count of magnetometer samples dropped due to invalid/NaN covariance or bad norm
uint32 mag_dropped_bad_cov

# Maximum observed time-sync slop between streams (ns). 0 if not tracked.
int64 time_sync_slop_max_ns

# -----------------------------
# Bootstrap / state
# -----------------------------

# True while in startup bootstrap phase
bool bootstrap_active

# Remaining bootstrap time (sec). 0 when inactive.
float64 bootstrap_remaining_sec

# True when anchored to base_link (reference pose captured)
bool anchored

# -----------------------------
# Steady / keyframe pipeline
# -----------------------------

# True when current steady detector window satisfies STEADY criteria
bool steady_detected

# Count of emitted steady segments
uint32 steady_segments

# Count of active keyframes
uint32 keyframes

# True when solver indicates insufficient tilt diversity
bool need_more_tilt

# True when solver indicates insufficient yaw diversity (when mag used)
bool need_more_yaw

# -----------------------------
# Magnetometer health
# -----------------------------

# True if mag reference is invalid / unavailable
bool mag_reference_invalid

# True if mag disturbance is detected (or mag constraints dropped)
bool mag_disturbance_detected

# Trace of current mag direction noise model R_m (unitless^2)
float64 mag_r_m_trace

# Count of mag factors dropped/ignored due to gating/disturbance
uint32 mag_factors_dropped

# -----------------------------
# Solver health
# -----------------------------

# Iterations used in the last optimizer update
int32 optimizer_iterations_last

# Step norm from the last optimizer update (rad)
float64 optimizer_step_norm_last

# Accel direction residual RMS (unitless ~ rad)
float64 accel_residual_rms

# Mag direction residual RMS (unitless ~ rad)
float64 mag_residual_rms

# -----------------------------
# Persistence
# -----------------------------

# Last successful save time (unix ns). 0 if never saved.
int64 last_save_unix_ns

# Save period configured (sec)
float64 save_period_sec

# Count of save failures since start/reset
uint32 save_fail_count
