/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{
class MonocularSlam;
}

namespace ROS
{
class MonocularSlamNode
{
public:
  MonocularSlamNode(rclcpp::Node& node);
  ~MonocularSlamNode();

  bool Initialize();
  void Deinitialize();

private:
  // ROS interface
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;

  // Video parameters
  std::unique_ptr<SLAM::MonocularSlam> m_monocularSlam;
};
} // namespace ROS
} // namespace OASIS
