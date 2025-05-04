################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# A single video‐frame “scene” containing all detected pose boxes
#
# Coordinates are in the same normalized image frame as in BoundingBox.
#

# Frame ID and timestamp of the source image
std_msgs/Header header

# One entry per detected pose
BoundingBox[] bounding_boxes
