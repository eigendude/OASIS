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

#include "apriltag/AprilTagVisualizer.h"

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace OASIS
{

class AprilTagVizNode
{
public:
  explicit AprilTagVizNode(rclcpp::Node& node);
  ~AprilTagVizNode();

  bool Initialize();
  void Deinitialize();

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void OnDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg);

  rclcpp::Node& m_node;
  rclcpp::Logger m_logger;

  AprilTagVisualizer m_visualizer;

  std::unique_ptr<image_transport::Subscriber> m_imageSubscription;
  std::unique_ptr<image_transport::Publisher> m_overlayPublisher;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      m_detectionSubscription;

  std::string m_systemId;
  std::string m_overlayMode;
  std::string m_imageTransport;
  double m_alpha{0.5};
};

} // namespace OASIS
