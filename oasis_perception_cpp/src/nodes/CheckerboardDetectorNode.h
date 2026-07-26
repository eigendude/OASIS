/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rclcpp
{
class Node;
} // namespace rclcpp

namespace OASIS
{
class CheckerboardDetectorNodeTestAccess;

namespace CALIBRATION
{
class CheckerboardDetector;
struct CheckerboardDetection;
} // namespace CALIBRATION

class CheckerboardDetectorNode
{
public:
  explicit CheckerboardDetectorNode(rclcpp::Node& node);
  ~CheckerboardDetectorNode();

  bool Initialize();
  void Deinitialize();

private:
  friend class CheckerboardDetectorNodeTestAccess;

  using DetectionCallback =
      std::function<void(const cv::Mat&, CALIBRATION::CheckerboardDetection&)>;
  using DebugImageCallback = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&,
                                                const cv::Mat&,
                                                const CALIBRATION::CheckerboardDetection&)>;
  using StatusCallback = std::function<void(bool)>;

  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg);
  void ProcessImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg);
  void PublishDebugImage(const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
                         const cv::Mat& gray,
                         const CALIBRATION::CheckerboardDetection& detection);
  void PublishStatus(bool found);
  void PublishStatusSafely(bool found);

  rclcpp::Node& m_node;

  std::unique_ptr<image_transport::Publisher> m_debugPublisher;
  std::unique_ptr<image_transport::Subscriber> m_imageSubscriber;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_statusPublisher;

  std::unique_ptr<CALIBRATION::CheckerboardDetector> m_detector;

  DetectionCallback m_detectionCallback;
  DebugImageCallback m_debugImageCallback;
  StatusCallback m_statusCallback;

  bool m_publishDebugImage{true};
  int64_t m_processingInterval{1};
  uint64_t m_frameCount{0};
};
} // namespace OASIS
