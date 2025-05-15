/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace oasis_perception
{
class PoseLandmarker
{
public:
  PoseLandmarker();
  ~PoseLandmarker();

  std::shared_ptr<sensor_msgs::msg::Image> OnImage(
      const std::shared_ptr<cv_bridge::CvImage const>& imagePtr);
};
} // namespace oasis_perception
