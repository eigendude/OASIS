/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CPUImageProc.h"

#include "api/ImageProc.h"

#include <opencv2/gapi/cpu/gcpukernel.hpp>
#include <opencv2/imgproc.hpp>

using namespace OASIS;
using namespace IMAGE;

// Find good features
// clang-format off
GAPI_OCV_KERNEL(GCPUGoodFeatures, GGoodFeatures)
{
  static void run(const cv::Mat& image,
                  const cv::Scalar& maxCorners,
                  double qualityLevel,
                  const cv::Scalar& minDistance,
                  const cv::Mat& mask,
                  int blockSize,
                  bool useHarrisDetector,
                  double k,
                  std::vector<cv::Point2f>& out)
  {
    cv::goodFeaturesToTrack(image,
                            out,
                            static_cast<int>(maxCorners[0]),
                            qualityLevel,
                            minDistance[0],
                            mask,
                            blockSize,
                            useHarrisDetector,
                            k);
  }
};
// clang-format on

cv::gapi::GKernelPackage kernels()
{
  static auto pkg = cv::gapi::kernels<GCPUGoodFeatures>();

  return pkg;
}
