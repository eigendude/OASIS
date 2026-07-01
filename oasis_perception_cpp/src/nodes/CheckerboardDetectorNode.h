/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <memory>

#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rclcpp
{
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace CALIBRATION
{
class CheckerboardDetector;
} // namespace CALIBRATION

class CheckerboardDetectorNode
{
public:
  explicit CheckerboardDetectorNode(rclcpp::Node& node);
  ~CheckerboardDetectorNode();

  bool Initialize();
  void Deinitialize();

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg);
  void PublishStatus(bool found);

  rclcpp::Node& m_node;

  std::unique_ptr<image_transport::Publisher> m_debugPublisher;
  std::unique_ptr<image_transport::Subscriber> m_imageSubscriber;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_statusPublisher;

  std::unique_ptr<CALIBRATION::CheckerboardDetector> m_detector;

  bool m_publishDebugImage{true};
  int64_t m_processingInterval{1};
  uint64_t m_frameCount{0};
};
} // namespace OASIS
