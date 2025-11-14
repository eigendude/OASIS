/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rclcpp
{
class NodeOptions;
} // namespace rclcpp

namespace oasis::perception
{
class MeshViewerNode : public rclcpp::Node
{
public:
  explicit MeshViewerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mesh_image_publisher_;

  double voxel_leaf_size_{0.05};
  double normal_search_radius_{0.1};
  double triangulation_search_radius_{0.2};
};
} // namespace oasis::perception
