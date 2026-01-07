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
# One update report for an AprilTagDetectionArray event
#

# Measurement timestamp and camera frame
std_msgs/Header header

# Update sequence index shared with per-detection reports
uint64 update_seq

# Total detections reported in the incoming message
uint32 num_detections_in_msg

# Total detections accepted by the filter
uint32 num_detections_accepted

# Per-detection update reports in sequential processing order
EkfAprilTagDetectionReport[] detections
