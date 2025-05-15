/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarker.h"

#include <opencv2/core.hpp>
// TODO: other includes

using namespace oasis_perception;

namespace
{
// TODO: Any constants go here
/*
constexpr unsigned int DOWNSCALED_maxWidth = 640;
constexpr unsigned int DOWNSCALED_maxHeight = 480;
*/
} // namespace

PoseLandmarker::PoseLandmarker() = default;
PoseLandmarker::~PoseLandmarker() = default;

std::shared_ptr<sensor_msgs::msg::Image> PoseLandmarker::OnImage(
    const std::shared_ptr<cv_bridge::CvImage const>& imagePtr)
{
  std::shared_ptr<sensor_msgs::msg::Image> outImage;

  cv::Mat inputImage = imagePtr->image;

  /* TODO: Replace with mediapipe
  const unsigned int maxWidth = DOWNSCALED_maxWidth;
  const unsigned int maxHeight = DOWNSCALED_maxHeight;

  // Compute scale factor to preserve aspect ratio
  double scaleWidth = static_cast<double>(maxWidth) / inputImage.cols;
  double scaleHeight = static_cast<double>(maxHeight) / inputImage.rows;

  double scale = std::min(scaleWidth, scaleHeight);

  // Ensure we only downscale if necessary (never upscale)
  scale = std::min(scale, 1.0);

  // Compute the target size
  cv::Size targetSize(static_cast<unsigned int>(inputImage.cols * scale),
                      static_cast<unsigned int>(inputImage.rows * scale));

  // Downscale the image using OpenCV
  cv::Mat resizedImage;
  cv::resize(inputImage, resizedImage, targetSize);

  // Convert the resized cv::Mat back into a ROS image message
  auto outMsg = cv_bridge::CvImage(msg->header, msg->encoding, resizedImage).toImageMsg();

  return outMsg;
  */

  return outImage;
}
