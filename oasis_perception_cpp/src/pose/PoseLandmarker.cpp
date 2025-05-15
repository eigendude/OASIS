/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarker.h"

#include <glog/logging.h>
#include <opencv2/core.hpp>

using namespace oasis_perception;

PoseLandmarker::PoseLandmarker() = default;

PoseLandmarker::~PoseLandmarker() = default;

bool PoseLandmarker::Initialize(const std::string& loggingName)
{
  google::InitGoogleLogging(loggingName.c_str());

  // Route to stderr
  FLAGS_logtostderr = true;
  FLAGS_alsologtostderr = true;

  // Full verbosity
  FLAGS_v = 3;
}

std::shared_ptr<sensor_msgs::msg::Image> PoseLandmarker::OnImage(
    const std::shared_ptr<cv_bridge::CvImage const>& imagePtr)
{
  std::shared_ptr<sensor_msgs::msg::Image> outImage;

  // TODO
  cv::Mat inputImage = imagePtr->image;

  return outImage;
}
