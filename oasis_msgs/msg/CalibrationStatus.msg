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
# Calibration status emitted by the ROS camera calibration pipeline while
# collecting and uploading camera intrinsics.
#
# Tracks sample accumulation, calibration execution, and CameraInfo publication
# so frontends can reflect progress and completion of the workflow.
#

# Number of calibration observations collected by camera_calibration's Calibrator.
# This counts each chessboard/ChArUco detection that passed the quality filters
# (spread of views, reprojection error, etc.) and is ready to contribute to the
# intrinsic solve.
uint32 sample_count

# True when the calibration target is detected in the current frame(s).
bool target_visible

# True once the Calibrator reports that the dataset is "good enough" to solve
# for intrinsics.  The upstream camera_calibration heuristics require a diverse
# set of target poses (tilt, pan, size in frame) before this flips to true.
bool good_enough

# True after calibration has successfully generated camera intrinsics and, for
# stereo rigs, the extrinsic transform
bool calibrated

# True once the computed camera info has been pushed to the camera driver
bool uploaded
