/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MapPointCloudBuilder.h"

#include "ViridisPaletteSampler.h"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace OASIS;
using namespace SLAM;

std::optional<sensor_msgs::msg::PointCloud2> MapPointCloudBuilder::BuildPointCloud(
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
    float maxHeight) const
{
  sensor_msgs::msg::PointCloud2 pointCloud;
  pointCloud.header = header;

  std::size_t pointCount = validMapPointCount;
  if (cameraValid)
    ++pointCount;
  if (arrowValid)
    ++pointCount;

  sensor_msgs::PointCloud2Modifier modifier(pointCloud);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
                                sensor_msgs::msg::PointField::UINT32);
  modifier.resize(pointCount);

  if (pointCount > 0)
  {
    sensor_msgs::PointCloud2Iterator<float> iterX(pointCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pointCloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pointCloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterR(pointCloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterG(pointCloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iterB(pointCloud, "b");

    const auto addPoint = [&](const Eigen::Vector3f& position, uint8_t red, uint8_t green,
                              uint8_t blue)
    {
      *iterX = position.x();
      *iterY = position.y();
      *iterZ = position.z();
      *iterR = red;
      *iterG = green;
      *iterB = blue;

      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterR;
      ++iterG;
      ++iterB;
    };

    const bool useDepthColors = cameraValid && (minDistance < maxDistance);
    const float distanceRange =
        useDepthColors ? std::max(maxDistance - minDistance, 1e-3F) : 0.0F;
    const float heightRange = std::max(maxHeight - minHeight, 1e-3F);

    for (const auto& renderPoint : pointsToRender)
    {
      Eigen::Vector3f color = Eigen::Vector3f::Zero();

      if (useDepthColors)
      {
        const float distance = (renderPoint.position - cameraPosition).norm();
        if (std::isfinite(distance))
        {
          const float depthFactor =
              std::clamp((distance - minDistance) / distanceRange, 0.0F, 1.0F);

          color = ViridisPaletteSampler::Sample(depthFactor);
        }
        else
        {
          const float heightFactor =
              std::clamp((renderPoint.position.y() - minHeight) / heightRange, 0.0F, 1.0F);
          color = Eigen::Vector3f((45.0F + heightFactor * 205.0F) / 255.0F,
                                  (80.0F + heightFactor * 140.0F) / 255.0F,
                                  (210.0F - heightFactor * 150.0F) / 255.0F);
        }
      }
      else
      {
        const float heightFactor =
            std::clamp((renderPoint.position.y() - minHeight) / heightRange, 0.0F, 1.0F);
        color = Eigen::Vector3f((45.0F + heightFactor * 205.0F) / 255.0F,
                                (80.0F + heightFactor * 140.0F) / 255.0F,
                                (210.0F - heightFactor * 150.0F) / 255.0F);
      }

      const float trackedBoost = renderPoint.tracked ? 1.1F : 0.95F;
      const auto encodeChannel = [&](float base)
      {
        const long value = std::lround(std::clamp(base * trackedBoost, 0.0F, 1.0F) * 255.0F);
        return static_cast<uint8_t>(std::clamp(value, 0L, 255L));
      };

      const uint8_t red = encodeChannel(color.x());
      const uint8_t green = encodeChannel(color.y());
      const uint8_t blue = encodeChannel(color.z());

      addPoint(renderPoint.position, red, green, blue);
    }

    if (cameraValid)
      addPoint(cameraPosition, 64, 160, 255);

    if (arrowValid)
      addPoint(arrowTip, 64, 160, 255);
  }

  pointCloud.is_dense = false;

  return pointCloud;
}

