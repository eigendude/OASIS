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
# The score summarizes how different the current frame is from the previous
# frame in a video stream. Consumers can use the score to detect scene cuts or
# trigger other logic when significant visual changes occur.
#

# Frame ID and timestamp of the source image
std_msgs/Header header

# Normalized scene score in the range [0.0, 1.0]
#
# A value closer to 1.0 indicates a large change between the current frame and
# its predecessor, while values near 0.0 indicate very little change.
float32 score

# Mean absolute frame difference (MAFD) for the current frame
#
# This value is calculated from the raw pixel intensity differences and is
# useful for debugging or calibrating scene score thresholds.
float32 mafd

# True when the score is based on at least two frames
#
# False indicates that the score could not be computed (for example, on the
# very first frame) because there is no previous frame for comparison.
bool is_valid
