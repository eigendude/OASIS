/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_transport
{
class ImageTransport;
class Publisher;
class Subscriber;
} // namespace image_transport

namespace ORB_SLAM3
{
class System;
}

namespace rclcpp
{
class Node;
}

namespace OASIS
{
namespace SLAM
{

class MonocularSlam
{
public:
  MonocularSlam(std::shared_ptr<rclcpp::Node> node, const std::string& imageTopic);
  ~MonocularSlam();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  // Logging parameters
  rclcpp::Logger m_logger;

  // ROS parameters
  std::unique_ptr<image_transport::ImageTransport> m_imgTransport;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;
};

} // namespace SLAM
} // namespace OASIS
