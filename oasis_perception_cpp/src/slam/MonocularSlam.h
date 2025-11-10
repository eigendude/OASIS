/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/image.hpp>

namespace ORB_SLAM3
{
class MapPoint;
class System;
} // namespace ORB_SLAM3

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace image_transport
{
class Publisher;
class Subscriber;
} // namespace image_transport

namespace cv
{
class Mat;
} // namespace cv

namespace OASIS
{
namespace SLAM
{

class MapVisualizer;

class MonocularSlam
{
public:
  MonocularSlam(rclcpp::Node& node, const std::string& mapImageTopic);
  ~MonocularSlam();

  // Lifecycle interface
  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;
};

} // namespace SLAM
} // namespace OASIS
