/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MapImageRenderer.h"
#include "MapPointCloudBuilder.h"
#include "MapPointRenderInfo.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace ORB_SLAM3
{
class MapPoint;
}

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace image_transport
{
class Publisher;
} // namespace image_transport

namespace OASIS
{
namespace SLAM
{

class MapVisualizer
{
public:
  MapVisualizer(rclcpp::Node& node,
                rclcpp::Logger& logger,
                const std::string& mapTopic,
                const std::string& mapImageTopic);
  ~MapVisualizer();

  void Publish(const std_msgs::msg::Header& header,
               const std::vector<ORB_SLAM3::MapPoint*>& trackedMapPoints,
               const Eigen::Vector3f& cameraPosition,
               const Eigen::Quaternionf& cameraOrientation);

  void Reset();

private:
  void PublishMapImage(const std_msgs::msg::Header& header,
                       const std::vector<MapPointRenderInfo>& renderPoints,
                       const Eigen::Vector3f& cameraPosition,
                       const Eigen::Quaternionf& cameraOrientation,
                       float maxDistance);

  rclcpp::Logger* m_logger = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_mapPublisher;
  std::unique_ptr<image_transport::Publisher> m_mapImagePublisher;
  MapPointCloudBuilder m_pointCloudBuilder;
  std::unique_ptr<MapImageRenderer> m_mapImageRenderer;
  std::unordered_map<const ORB_SLAM3::MapPoint*, Eigen::Vector3f> m_mapPointPositions;
};

} // namespace SLAM
} // namespace OASIS
