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
# Compact per-update EKF report for diagnostics and replay visibility
#

# Unknown or unclassified update kind
uint8 UPDATE_UNKNOWN = 0

# IMU-driven propagation step
uint8 UPDATE_PREDICT = 1

# IMU attitude observation update
uint8 UPDATE_IMU_ATTITUDE = 2

# Gravity-direction update
uint8 UPDATE_GRAVITY = 3

# AprilTag landmark initialization
uint8 UPDATE_APRILTAG_INIT = 4

# AprilTag reprojection update against an existing landmark
uint8 UPDATE_APRILTAG_REPROJECTION = 5

# Update timestamp and primary output frame
std_msgs/Header header

# Update kind for this EKF event
uint8 update_kind

# True when the update was accepted into the EKF
bool accepted

# Source frame for the measurement packet when relevant
string source_frame_id

# Norm of the innovation used by this update
#
# Units: implementation-defined
#
float64 innovation_norm

# Mahalanobis distance for gating and diagnostics
float64 mahalanobis_distance

# Acceptance gate threshold used for this update
float64 gate_threshold

# Short summary of the state block touched by this update
string affected_state

# AprilTag family for tag-related updates
string family

# AprilTag id for tag-related updates
int32 id

# Detector decision margin for tag-related updates
float64 decision_margin

# Reprojection error summary for tag-related updates
#
# Units: px
#
float64 reprojection_error_px
