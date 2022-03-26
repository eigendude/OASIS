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

namespace bgslibrary
{
namespace algorithms
{
class IBGS;
}
} // namespace bgslibrary

namespace image_transport
{
class ImageTransport;
class Publisher;
class Subscriber;
} // namespace image_transport

namespace rclcpp
{
class Node;
}

namespace OASIS
{
namespace IMAGE
{

class BackgroundModeler
{
public:
  BackgroundModeler(std::shared_ptr<rclcpp::Node> node,
                    const std::string& imageTopic,
                    const std::string& foregroundTopic,
                    const std::string& backgroundTopic);
  ~BackgroundModeler();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  // Logging parameters
  rclcpp::Logger m_logger;

  // Image parameters
  std::unique_ptr<image_transport::ImageTransport> m_imgTransport;
  std::unique_ptr<image_transport::Publisher> m_imgPublisherForeground;
  std::unique_ptr<image_transport::Publisher> m_imgPublisherBackground;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  std::unique_ptr<bgslibrary::algorithms::IBGS> m_bgsPackage;
};

} // namespace IMAGE
} // namespace OASIS
