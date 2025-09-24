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

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace bgslibrary
{
namespace algorithms
{
class IBGS;
} // namespace algorithms
} // namespace bgslibrary

namespace image_transport
{
class Publisher;
class Subscriber;
} // namespace image_transport

namespace OASIS
{

class BackgroundSubtractorNode
{
public:
  explicit BackgroundSubtractorNode(rclcpp::Node& node);
  ~BackgroundSubtractorNode();

  bool Initialize();
  void Deinitialize();

private:
  void ReceiveImage(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);

  rclcpp::Node& m_node;

  std::string m_zoneId;
  std::unique_ptr<image_transport::Publisher> m_imgPublisherForeground;
  std::unique_ptr<image_transport::Publisher> m_imgPublisherSubtracted;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;

  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackage;
};

} // namespace OASIS
