################################################################################
#
#  Copyright (C) 2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# One update report for a single measurement model instance
#

# Measurement timestamp and frame
std_msgs/Header header

# Sensor name such as "magnetic_field" or "apriltags"
string sensor

# Measurement frame id such as "{M}" or "{C}"
string frame_id

# Monotonic update sequence index for the EKF node
uint64 update_seq

# Stable per-stamp ordering index after deterministic sorting
uint32 update_index_in_stamp

# True when the update was accepted by the filter
bool accepted

# Reason for rejection or empty when accepted
string reject_reason

# Dimension of the measurement vector
uint32 z_dim

# Measurement vector
float64[] z

# Predicted measurement vector
float64[] z_hat

# Innovation computed as z minus z_hat
float64[] nu

# Measurement noise covariance actually used
Matrix R

# Predicted innovation covariance without measurement noise
Matrix S_hat

# Total innovation covariance including measurement noise
Matrix S

# Squared Mahalanobis distance of the innovation
float64 maha_d2

# Squared gating threshold value, or zero when unused
float64 gate_d2_threshold

# Root mean square reprojection error in pixels for vision updates
float64 reproj_rms_px
