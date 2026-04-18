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
# One world-frame AprilTag landmark tracked by the EKF
#

# AprilTag family string used with id as the landmark key
#
# Landmark identity is the pair (family, id)
#
string family

# AprilTag id within the tag family
int32 id

# True when pose contains an initialized world-frame landmark estimate
bool initialized

# World-frame landmark pose and full 6x6 covariance
geometry_msgs/PoseWithCovariance pose

# Most recent reprojection error summary for this landmark
#
# Units: px
#
float64 last_reprojection_error_px
