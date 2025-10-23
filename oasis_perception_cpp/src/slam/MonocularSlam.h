/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace image_transport
{
class Publisher;
class Subscriber;
} // namespace image_transport

namespace ORB_SLAM3
{
class MapPoint;
class System;
} // namespace ORB_SLAM3

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace cv
{
class Mat;
} // namespace cv

namespace OASIS
{
namespace SLAM
{

class MonocularSlam
{
public:
  MonocularSlam(rclcpp::Node& node,
                const std::string& mapTopic,
                const std::string& debugTopic = "");
  ~MonocularSlam();

  // Lifecycle interface
  bool Initialize(const std::string& vocabularyFile, const std::string& settingsFile);
  void Deinitialize();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

private:
  void PublishMapVisualization(const std_msgs::msg::Header& header,
                               const std::vector<ORB_SLAM3::MapPoint*>& trackedMapPoints,
                               const Eigen::Vector3f& cameraPosition,
                               const Eigen::Quaternionf& cameraOrientation);
  void PublishDebugImage(const std_msgs::msg::Header& header, const cv::Mat& debugImage);

  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;
  std::unique_ptr<image_transport::Publisher> m_mapPublisher;
  std::unique_ptr<image_transport::Publisher> m_debugPublisher;

  std::unordered_map<const ORB_SLAM3::MapPoint*, Eigen::Vector3f> m_mapPointPositions;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;
};

} // namespace SLAM
} // namespace OASIS
