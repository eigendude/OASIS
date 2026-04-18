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
# World-frame AprilTag landmark map published by the EKF
#

# Map timestamp
#
# header.frame_id is expected to be world
#
std_msgs/Header header

# Current world-frame landmark entries
AprilTagMapEntry[] entries
