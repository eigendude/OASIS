/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "OrbImuSettingsValidator.h"

#include <cmath>
#include <exception>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
bool IsFiniteDouble(double value)
{
  return std::isfinite(value);
}

std::string FormatVector3(double x, double y, double z)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << "[" << x << ", " << y << ", " << z << "]";
  return stream.str();
}

bool ValidatePositiveImuParameter(const cv::FileStorage& settings,
                                  std::string_view key,
                                  rclcpp::Logger& logger)
{
  const std::string keyString{key};
  const cv::FileNode node = settings[keyString];
  if (node.empty())
  {
    RCLCPP_ERROR(logger, "ORB-SLAM3 IMU settings missing required parameter '%s'",
                 keyString.c_str());
    return false;
  }

  if (!node.isReal() && !node.isInt())
  {
    RCLCPP_ERROR(logger, "ORB-SLAM3 IMU parameter '%s' is not numeric", keyString.c_str());
    return false;
  }

  const double value = static_cast<double>(node);
  if (!IsFiniteDouble(value) || value <= 0.0)
  {
    RCLCPP_ERROR(logger,
                 "ORB-SLAM3 IMU parameter '%s' must be finite and positive "
                 "(value=%.9g)",
                 keyString.c_str(), value);
    return false;
  }

  return true;
}

bool ValidateImuTransform(const cv::FileStorage& settings, rclcpp::Logger& logger)
{
  const cv::FileNode node = settings["IMU.T_b_c1"];
  if (node.empty())
  {
    RCLCPP_ERROR(logger, "ORB-SLAM3 IMU settings missing required transform 'IMU.T_b_c1'");
    return false;
  }

  cv::Mat rawTransform;
  try
  {
    rawTransform = node.mat();
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(logger, "Failed to read ORB-SLAM3 IMU transform 'IMU.T_b_c1': %s", ex.what());
    return false;
  }

  if (rawTransform.rows != 4 || rawTransform.cols != 4)
  {
    RCLCPP_ERROR(logger,
                 "ORB-SLAM3 IMU transform 'IMU.T_b_c1' must be 4x4 "
                 "(rows=%d, cols=%d)",
                 rawTransform.rows, rawTransform.cols);
    return false;
  }

  cv::Mat transform;
  try
  {
    rawTransform.convertTo(transform, CV_64F);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(logger, "Failed to convert ORB-SLAM3 IMU transform to double: %s", ex.what());
    return false;
  }

  for (int row = 0; row < transform.rows; ++row)
  {
    for (int column = 0; column < transform.cols; ++column)
    {
      const double value = transform.at<double>(row, column);
      if (!IsFiniteDouble(value))
      {
        RCLCPP_ERROR(logger,
                     "ORB-SLAM3 IMU transform contains non-finite value at "
                     "(row=%d, col=%d)",
                     row, column);
        return false;
      }
    }
  }

  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  for (int row = 0; row < 3; ++row)
  {
    for (int column = 0; column < 3; ++column)
      rotation(row, column) = transform.at<double>(row, column);

    translation(row) = transform.at<double>(row, 3);
  }

  if (!translation.allFinite())
  {
    RCLCPP_ERROR(logger, "ORB-SLAM3 IMU transform translation contains NaN or Inf");
    return false;
  }

  const double determinant = rotation.determinant();
  if (!IsFiniteDouble(determinant) || std::abs(determinant - 1.0) > 1.0e-3)
  {
    RCLCPP_ERROR(logger,
                 "ORB-SLAM3 IMU transform rotation determinant must be near +1 "
                 "(det=%.9g)",
                 determinant);
    return false;
  }

  const Eigen::Matrix3d orthonormalError =
      rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
  if (!orthonormalError.allFinite() || orthonormalError.norm() > 1.0e-3)
  {
    RCLCPP_ERROR(logger,
                 "ORB-SLAM3 IMU transform rotation is not orthonormal "
                 "(error_norm=%.9g)",
                 orthonormalError.norm());
    return false;
  }

  const double bottomRowX = transform.at<double>(3, 0);
  const double bottomRowY = transform.at<double>(3, 1);
  const double bottomRowZ = transform.at<double>(3, 2);
  const double bottomRowW = transform.at<double>(3, 3);
  if (std::abs(bottomRowX) > 1.0e-6 || std::abs(bottomRowY) > 1.0e-6 ||
      std::abs(bottomRowZ) > 1.0e-6 || std::abs(bottomRowW - 1.0) > 1.0e-6)
  {
    RCLCPP_ERROR(logger,
                 "ORB-SLAM3 IMU transform bottom row must be [0, 0, 0, 1] "
                 "(row=[%.9g, %.9g, %.9g, %.9g])",
                 bottomRowX, bottomRowY, bottomRowZ, bottomRowW);
    return false;
  }

  RCLCPP_INFO(logger,
              "Validated ORB-SLAM3 IMU transform 'IMU.T_b_c1' "
              "(det=%.6f, translation=%s)",
              determinant,
              FormatVector3(translation.x(), translation.y(), translation.z()).c_str());

  return true;
}
} // namespace

bool OASIS::SLAM::ValidateOrbImuSettings(const std::string& settingsFile, rclcpp::Logger& logger)
{
  cv::FileStorage settings;
  try
  {
    settings.open(settingsFile, cv::FileStorage::READ);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(logger, "Failed to open ORB-SLAM3 settings file '%s': %s", settingsFile.c_str(),
                 ex.what());
    return false;
  }

  if (!settings.isOpened())
  {
    RCLCPP_ERROR(logger, "Failed to open ORB-SLAM3 settings file '%s'", settingsFile.c_str());
    return false;
  }

  bool valid = true;
  valid = ValidatePositiveImuParameter(settings, "IMU.NoiseGyro", logger) && valid;
  valid = ValidatePositiveImuParameter(settings, "IMU.NoiseAcc", logger) && valid;
  valid = ValidatePositiveImuParameter(settings, "IMU.GyroWalk", logger) && valid;
  valid = ValidatePositiveImuParameter(settings, "IMU.AccWalk", logger) && valid;
  valid = ValidatePositiveImuParameter(settings, "IMU.Frequency", logger) && valid;
  valid = ValidateImuTransform(settings, logger) && valid;

  if (valid)
    RCLCPP_INFO(logger, "Validated ORB-SLAM3 IMU settings in '%s'", settingsFile.c_str());

  return valid;
}
