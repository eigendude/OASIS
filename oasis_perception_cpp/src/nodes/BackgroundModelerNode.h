/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>
#include <string>

#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace bgslibrary
{
namespace algorithms
{
class IBGS;
} // namespace algorithms
} // namespace bgslibrary

namespace OASIS
{

class BackgroundModelerNode
{
public:
  explicit BackgroundModelerNode(rclcpp::Node& node);
  ~BackgroundModelerNode();

  bool Start();
  void Stop();

private:
  // ROS interface
  void ReceiveImage(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);

  // Construction parameters
  rclcpp::Node& m_node;
  rclcpp::Node::SharedPtr m_nodeShared;

  // ROS parameters
  std::string m_zoneId;
  std::unique_ptr<image_transport::ImageTransport> m_imgTransport;
  image_transport::Publisher m_imgPublisherBackground;
  image_transport::Subscriber m_imgSubscriber;

  // Background subtractors
  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackage;
};

} // namespace OASIS
