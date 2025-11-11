/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CPUVideo.h"

#include "gapi/Video.h"

#include <opencv2/gapi/cpu/gcpukernel.hpp>
#include <opencv2/gapi/cpu/imgproc.hpp>

namespace OASIS::VIDEO
{

// Predict points for optical flow
// clang-format off
GAPI_OCV_KERNEL(GCPUPredictPoints, GPredictPoints)
{
  static void run(const std::vector<std::vector<cv::Point2f>>& pointHistory,
                  std::vector<cv::Point2f>& predictedPoints)
  {
    predictedPoints.resize(pointHistory[0].size());

    // TODO
    predictedPoints = pointHistory.back();
  }
};
// clang-format on

cv::gapi::GKernelPackage kernels()
{
  static auto pkg = cv::gapi::kernels<GCPUPredictPoints>();

  return pkg;
}

} // namespace OASIS::VIDEO
