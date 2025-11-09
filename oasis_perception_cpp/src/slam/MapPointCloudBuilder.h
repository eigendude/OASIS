/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "MapPointRenderInfo.h"

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace OASIS
{
namespace SLAM
{

class MapPointCloudBuilder
{
public:
  std::optional<sensor_msgs::msg::PointCloud2> BuildPointCloud(
      const std_msgs::msg::Header& header,
      const std::vector<MapPointRenderInfo>& pointsToRender,
      std::size_t validMapPointCount,
      bool cameraValid,
      const Eigen::Vector3f& cameraPosition,
      bool arrowValid,
      const Eigen::Vector3f& arrowTip,
      float minDistance,
      float maxDistance,
      float minHeight,
      float maxHeight) const;
};

} // namespace SLAM
} // namespace OASIS

