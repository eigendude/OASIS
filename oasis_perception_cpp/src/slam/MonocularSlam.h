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

#include <sensor_msgs/msg/image.hpp>

namespace image_transport
{
class Subscriber;
} // namespace image_transport

namespace ORB_SLAM3
{
class System;
}

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{

class MonocularSlam
{
public:
  MonocularSlam(rclcpp::Node& node);
  ~MonocularSlam();

  // Lifecycle interface
  bool Initialize();
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
