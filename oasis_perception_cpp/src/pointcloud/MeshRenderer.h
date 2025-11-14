/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace OASIS
{
class MeshRenderer
{
public:
  MeshRenderer(rclcpp::Logger logger,
               double voxelLeafSize,
               double normalSearchRadius,
               double triangulationSearchRadius);
  ~MeshRenderer();

  sensor_msgs::msg::Image::SharedPtr Render(const sensor_msgs::msg::PointCloud2& pointCloud) const;

private:
  // ROS parameters
  rclcpp::Logger m_logger;

  // Mesh parameters
  double m_voxelLeafSize;
  double m_normalSearchRadius;
  double m_triangulationSearchRadius;
};
} // namespace OASIS
