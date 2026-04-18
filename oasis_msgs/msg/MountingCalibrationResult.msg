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
# Solved fixed mounting transform used by AHRS and EKF
#

# Result timestamp and solved parent frame
std_msgs/Header header

# TF parent frame for the solved fixed transform
string parent_frame_id

# TF child frame for the solved fixed transform
string child_frame_id

# Solved fixed transform T_BI that maps imu_link into base_link
#
# Translation is expected to be zero by current policy
#
geometry_msgs/Transform transform

# True when the solver produced a usable fixed transform
bool success

# Boot calibration window duration used for this solve
#
# Units: s
#
float64 boot_window_duration_sec

# Number of accepted samples used in the final solve
#
# Units: samples
#
uint32 accepted_sample_count

# Gravity residual summary for the solved tilt fit
#
# Units: implementation-defined
#
float64 gravity_residual_metric

# Fused-attitude residual summary for the solved tilt fit
#
# Units: implementation-defined
#
float64 fused_attitude_residual_metric

# Short implementation-facing note about the solved result
string status_text
