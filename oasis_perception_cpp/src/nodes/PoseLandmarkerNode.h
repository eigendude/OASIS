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
class Node;
}

namespace oasis_perception
{
class PoseLandmarker;

class PoseLandmarkerNode
{
public:
  PoseLandmarkerNode(rclcpp::Node& node);
  ~PoseLandmarkerNode();

  bool Start();
  void Stop();

private:
  // ROS interface
  void OnImage(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);

  // Construction parameters
  rclcpp::Node& m_node;

  // ROS parameters
  image_transport::Subscriber m_subscriber;
  image_transport::Publisher m_publisher;

  // Pose landmarker instance
  std::unique_ptr<PoseLandmarker> m_poseLandmarker;
};
} // namespace oasis_perception
