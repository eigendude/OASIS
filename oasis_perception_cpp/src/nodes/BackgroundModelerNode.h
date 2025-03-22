/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

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
class ImageTransport;
class Publisher;
class Subscriber;
} // namespace image_transport

namespace OASIS
{

class BackgroundModelerNode : public rclcpp::Node
{
public:
  BackgroundModelerNode();
  ~BackgroundModelerNode() override;

  void Initialize();

private:
  // ROS interface
  void ReceiveImage(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);

  // ROS parameters
  std::unique_ptr<image_transport::ImageTransport> m_imgTransport;
  std::unique_ptr<image_transport::Publisher> m_imgPublisherBackground;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;

  // Background subtractors
  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackage;
};

} // namespace OASIS
