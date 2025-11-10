/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace rclcpp
{
class Logger;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{

struct CameraModel
{
  int width{0};
  int height{0};
  float fx{0.0f};
  float fy{0.0f};
  float cx{0.0f};
  float cy{0.0f};
};

bool LoadCameraModel(const std::string& settingsFile, CameraModel& model, rclcpp::Logger& logger);

} // namespace SLAM
} // namespace OASIS
