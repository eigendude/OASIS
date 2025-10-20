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
# Scene change score for a single video frame
#

# Frame ID and timestamp of the source image
std_msgs/Header header

# Normalized scene score in the range [0.0, 1.0]
float32 score

# Mean absolute frame difference (MAFD) for the current frame
float32 mafd

# True when the score is based on at least two frames
bool is_valid
