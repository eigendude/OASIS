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
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
                  int maxHeight,
                  const std::string& cameraInfoTopic,
                  const std::string& downscaledCameraInfoTopic);
  ~ImageDownscaler();

  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void ReceiveCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& msg);

private:
  std::pair<int, int> CalculateTargetDimensions(int width, int height) const;

  rclcpp::Logger m_logger;
  std::shared_ptr<rclcpp::Node> m_node;
  std::unique_ptr<image_transport::Publisher> m_downscaledPublisher;
  std::unique_ptr<image_transport::Subscriber> m_imageSubscriber;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_cameraInfoPublisher;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cameraInfoSubscriber;
  int m_maxWidth{0};
  int m_maxHeight{0};
};

} // namespace IMAGE
} // namespace OASIS
