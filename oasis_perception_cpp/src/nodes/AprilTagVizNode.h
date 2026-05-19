/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "apriltag/AprilTagVisualizer.h"

#include <memory>
#include <mutex>
#include <string>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_transport
{
class Publisher;
class Subscriber;
} // namespace image_transport

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
  void OnImage(const sensor_msgs::msg::Image& imageMsg);
  void OnDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg);

  rclcpp::Node& m_node;
  rclcpp::Logger m_logger;

  AprilTagVisualizer m_visualizer;

  // Publishers
  std::unique_ptr<image_transport::Publisher> m_overlayPublisher;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr m_imageCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr m_detectionsCallbackGroup;

  // Subscribers
  std::unique_ptr<image_transport::Subscriber> m_imageSubscription;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      m_detectionSubscription;

  std::mutex m_visualizerMutex;

  std::string m_systemId;
  std::string m_imageTransport;
};

} // namespace OASIS
