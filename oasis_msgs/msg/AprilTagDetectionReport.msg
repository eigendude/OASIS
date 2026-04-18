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
# OASIS AprilTag detection report with estimator-specific outcomes
#

# Raw upstream AprilTag detection
apriltag_msgs/AprilTagDetection detection

# True when this detection contributed to the estimator update
bool accepted

# True when this detection initialized a new world-frame landmark
bool initialized_landmark

# Reprojection error summary for this detection
#
# Units: px
#
float64 reprojection_error_px

# Mahalanobis distance for the accepted or rejected detection update
float64 mahalanobis_distance

# Gate threshold used for this detection
float64 gate_threshold

# True when world_pose is populated for this detection
bool has_world_pose

# Optional world-frame pose summary for the detected tag
geometry_msgs/PoseWithCovariance world_pose
