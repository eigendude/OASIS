/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CameraModel.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

#include <opencv2/core.hpp>
#include <rclcpp/logging.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
constexpr std::string_view CAMERA_WIDTH_KEY = "Camera.width";
constexpr std::string_view CAMERA_HEIGHT_KEY = "Camera.height";
constexpr std::string_view CAMERA_FX_KEY = "Camera.fx";
constexpr std::string_view CAMERA_FY_KEY = "Camera.fy";
constexpr std::string_view CAMERA_CX_KEY = "Camera.cx";
constexpr std::string_view CAMERA_CY_KEY = "Camera.cy";

bool ValidateCameraModel(const CameraModel& model)
{
  return model.width > 0 && model.height > 0 && std::isfinite(model.fx) && model.fx > 0.0f &&
         std::isfinite(model.fy) && model.fy > 0.0f && std::isfinite(model.cx) &&
         std::isfinite(model.cy);
}

} // namespace

bool SLAM::LoadCameraModel(const std::string& settingsFile,
                           CameraModel& model,
                           rclcpp::Logger& logger)
{
  cv::FileStorage settings(settingsFile, cv::FileStorage::READ);
  if (!settings.isOpened())
  {
    RCLCPP_ERROR(logger, "Failed to open settings file: %s", settingsFile.c_str());
    return false;
  }

  settings[std::string(CAMERA_WIDTH_KEY)] >> model.width;
  settings[std::string(CAMERA_HEIGHT_KEY)] >> model.height;
  settings[std::string(CAMERA_FX_KEY)] >> model.fx;
  settings[std::string(CAMERA_FY_KEY)] >> model.fy;
  settings[std::string(CAMERA_CX_KEY)] >> model.cx;
  settings[std::string(CAMERA_CY_KEY)] >> model.cy;

  if (!ValidateCameraModel(model))
  {
    RCLCPP_ERROR(logger,
                 "Invalid camera model read from settings file %s (width=%d height=%d fx=%f fy=%f "
                 "cx=%f cy=%f)",
                 settingsFile.c_str(), model.width, model.height, model.fx, model.fy, model.cx,
                 model.cy);
    return false;
  }

  RCLCPP_INFO(logger, "Loaded camera model: width=%d height=%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
              model.width, model.height, model.fx, model.fy, model.cx, model.cy);

  return true;
}
