/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <memory>
#include <string>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/header.hpp>
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
  void OnDetections(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr& msg);

  static std::array<double, 2> Project(const std::array<double, 9>& homography,
                                       const std::array<double, 2>& pointInCamera);

  rclcpp::Node& m_node;
  rclcpp::Logger m_logger;

  std::unique_ptr<image_transport::Subscriber> m_imageSubscription;
  std::unique_ptr<image_transport::Publisher> m_overlayPublisher;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
      m_detectionSubscription;

  std::string m_systemId;
  std::string m_overlayMode;
  std::string m_imageTransport;
  double m_alpha{0.5};

  cv::Mat m_latestImage;
  cv::Mat m_overlayImage;
  cv::Mat m_mergedImage;

  std_msgs::msg::Header m_latestHeader;
  std::string m_latestEncoding;
};

} // namespace OASIS
