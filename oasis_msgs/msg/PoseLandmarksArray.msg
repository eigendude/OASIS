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
# The full "scene" of zero‑or‑more poses in a frame
#

# Timestamp and frame_id of source image
std_msgs/Header header

# One entry per detected person
PoseLandmarks[] poses
