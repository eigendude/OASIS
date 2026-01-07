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
# Per-tag report for a single AprilTag detection update
#

# AprilTag family name such as "tag36h11"
string family

# AprilTag identifier within the family
int32 id

# Index in the incoming AprilTagDetectionArray message
uint32 det_index_in_msg

# Index after sorting by family and id for sequential processing
uint32 order_index

# Embedded update report with z_dim equal to 8 for corners
EkfUpdateReport report
