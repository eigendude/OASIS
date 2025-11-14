/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rclcpp
{
class Node;
} // namespace rclcpp

namespace OASIS
{
class MeshViewerNode
{
public:
  explicit MeshViewerNode(rclcpp::Node& node);
  ~MeshViewerNode();

  bool Initialize();
  void Deinitialize();

private:
  void OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  rclcpp::Node& m_node;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudSubscription;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_meshImagePublisher;

  double m_voxelLeafSize;
  double m_normalSearchRadius;
  double m_triangulationSearchRadius;
};
} // namespace OASIS
