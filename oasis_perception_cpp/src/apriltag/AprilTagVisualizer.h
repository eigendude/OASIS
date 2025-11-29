/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <memory>
#include <string>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace OASIS
{

class AprilTagVisualizer
{
public:
  AprilTagVisualizer(const rclcpp::Logger& logger, const rclcpp::Clock::SharedPtr& clock);
  ~AprilTagVisualizer();

  void SetOverlayMode(const std::string& overlayMode);
  void SetAlpha(double alpha);

  sensor_msgs::msg::Image::SharedPtr ProcessImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  sensor_msgs::msg::Image::SharedPtr ProcessDetections(
      const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg);

private:
  static std::array<double, 2> Project(const std::array<double, 9>& homography,
                                       const std::array<double, 2>& pointInCamera);

  sensor_msgs::msg::Image::SharedPtr CreateOutputMessage();

  rclcpp::Logger m_logger;
  rclcpp::Clock::SharedPtr m_clock;

  std::string m_overlayMode{ "axes" };
  double m_alpha{0.5};

  cv::Mat m_latestImage;
  cv::Mat m_overlayImage;
  cv::Mat m_mergedImage;

  std_msgs::msg::Header m_latestHeader;
  std::string m_latestEncoding;
};

} // namespace OASIS
