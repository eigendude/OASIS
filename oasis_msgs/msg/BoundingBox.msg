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
# Normalized 2D bounding box for a detected pose
#
# Coordinates are expressed in the image frame where:
#
#  - The origin (0,0) is the top‑left corner of the image
#  - X increases to the right, Y increases downward
#  - All values are normalized to [0..1] relative to image dimensions
#

# Center X coordinate, normalized [0..1]
float32 x_center

# Center Y coordinate, normalized [0..1]
float32 y_center

# Box width, normalized [0..1]
float32 width

# Box height, normalized [0..1]
float32 height
