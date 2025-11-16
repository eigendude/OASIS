/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_transport
{
class CameraPublisher;
class CameraSubscriber;
class TransportHints;
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
                  std::optional<unsigned int> outputWidth,
                  std::optional<unsigned int> outputHeight,
                  std::optional<unsigned int> maxWidth,
                  std::optional<unsigned int> maxHeight);
  ~ImageDownscaler();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfoPtr);
  void ReceiveCameraInfo();

private:
  std::pair<unsigned int, unsigned int> CalculateTargetDimensions(unsigned int width,
                                                                  unsigned int height) const;

  rclcpp::Logger m_logger;
  std::shared_ptr<rclcpp::Node> m_node;
  std::unique_ptr<image_transport::CameraPublisher> m_downscaledPublisher;
  std::unique_ptr<image_transport::CameraSubscriber> m_cameraSubscriber;
  std::optional<unsigned int> m_outputWidth;
  std::optional<unsigned int> m_outputHeight;
  std::optional<unsigned int> m_maxWidth;
  std::optional<unsigned int> m_maxHeight;
};

} // namespace IMAGE
} // namespace OASIS
