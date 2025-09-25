/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ImageProc.h"

#include <opencv2/gapi/imgproc.hpp>

namespace OASIS::IMAGE
{

cv::GMat RGBA2Gray(const cv::GMat& rgbaImage)
{
  return cv::gapi::RGBA2Gray(rgbaImage);
}

cv::GArray<cv::Point2f> GoodFeaturesToTrack(const cv::GMat& grayscaleImage,
                                            const cv::GScalar& maxFeatures,
                                            const cv::GScalar& minDistance,
                                            double qualityLevel,
                                            const cv::Mat& mask,
                                            int blockSize,
                                            bool useHarrisDetector,
                                            double k)
{
  return GGoodFeatures::on(grayscaleImage, maxFeatures, qualityLevel, minDistance, mask, blockSize,
                           useHarrisDetector, k);
}

} // namespace OASIS::IMAGE
