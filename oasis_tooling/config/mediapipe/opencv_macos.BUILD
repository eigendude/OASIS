# Copyright (C) 2026 Garrett Brown
# This file is part of OASIS - https://github.com/eigendude/OASIS
# SPDX-License-Identifier: Apache-2.0
# See the file LICENSE.txt for more information.

load("@rules_cc//cc:cc_library.bzl", "cc_library")

# The local repository root is the OpenCV installation shared with OASIS
cc_library(
    name = "opencv",
    srcs = [
        "lib/libopencv_calib3d.dylib",
        "lib/libopencv_core.dylib",
        "lib/libopencv_features2d.dylib",
        "lib/libopencv_highgui.dylib",
        "lib/libopencv_imgcodecs.dylib",
        "lib/libopencv_imgproc.dylib",
        "lib/libopencv_video.dylib",
        "lib/libopencv_videoio.dylib",
    ],
    hdrs = glob(["include/opencv4/opencv2/**/*.h*"]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"],
)
