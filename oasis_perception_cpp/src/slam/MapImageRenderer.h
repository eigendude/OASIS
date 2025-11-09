/*
 * Copyright (C) 2025 Garrett Brown
 * This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 * SPDX-License-Identifier: Apache-2.0
 * See the file LICENSE.txt for more information.
 */

#pragma once

#include "MapPointRenderInfo.h"

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace OASIS
{
namespace SLAM
{

class MapImageRenderer
{
public:
  MapImageRenderer();

  sensor_msgs::msg::Image::SharedPtr RenderImage(
      const std_msgs::msg::Header& header,
      const std::vector<MapPointRenderInfo>& renderPoints,
      const Eigen::Vector3f& cameraPosition,
      const Eigen::Quaternionf& cameraOrientation,
      float maxDistance);

private:
  struct DepthEMA
  {
    float zMin = 0.0F;
    float zMax = 1.0F;
    bool init = false;

    void Update(float newMin, float newMax, float alpha = 0.1F);
  };

  void ApplyFisheyeEffect(cv::Mat& image);

  DepthEMA m_depthEma;
  cv::Mat m_mapX;
  cv::Mat m_mapY;
  cv::Size m_cachedSize;
};

} // namespace SLAM
} // namespace OASIS

