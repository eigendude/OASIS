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
# One AprilTag processing report for each image / apriltags message
#

# Frame processed successfully
uint8 STATUS_OK = 0

# Frame had no cached camera intrinsics
uint8 STATUS_NO_INTRINSICS = 1

# Frame was processed but no tags were visible
uint8 STATUS_NO_DETECTIONS = 2

# Frame was rejected by policy
uint8 STATUS_REJECTED = 3

# Timestamp of the source image or apriltags packet
std_msgs/Header header

# Camera frame used for the AprilTag packet
string camera_frame_id

# True when camera intrinsics were available for this frame
bool had_intrinsics

# Number of raw detections in the source frame
#
# Units: detections
#
uint32 detection_count

# Per-frame processing status
uint8 status

# Per-detection reports for this frame
#
# An empty array is valid when no tags are visible
#
AprilTagDetectionReport[] detections

# True when base_pose_world is populated for this frame
bool has_base_pose_world

# Optional world-frame base pose summary for this frame
geometry_msgs/PoseWithCovariance base_pose_world

# Short implementation-facing frame status text
string status_text
