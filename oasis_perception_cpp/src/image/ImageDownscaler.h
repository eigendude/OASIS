################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

#pragma once

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_transport
{
class Publisher;
class Subscriber;
} // namespace image_transport

namespace rclcpp
{
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace IMAGE
{

class ImageDownscaler
{
public:
  ImageDownscaler(std::shared_ptr<rclcpp::Node> node,
                  const std::string& imageTopic,
                  const std::string& downscaledTopic,
                  const std::string& imageTransport,
                  int maxWidth,
                  int maxHeight);
  ~ImageDownscaler();

  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  rclcpp::Logger m_logger;
  std::shared_ptr<rclcpp::Node> m_node;
  std::unique_ptr<image_transport::Publisher> m_downscaledPublisher;
  std::unique_ptr<image_transport::Subscriber> m_imageSubscriber;
  int m_maxWidth{0};
  int m_maxHeight{0};
};

} // namespace IMAGE
} // namespace OASIS

