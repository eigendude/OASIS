/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MapVisualizer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>

#include <MapPoint.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
bool ExtractMapPointPosition(const ORB_SLAM3::MapPoint* mapPoint, Eigen::Vector3f& position)
{
  if (mapPoint == nullptr)
    return false;

  if (const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->isBad())
    return false;

  position = const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->GetWorldPos();

  return std::isfinite(position.x()) && std::isfinite(position.y()) && std::isfinite(position.z());
}

} // namespace

MapVisualizer::MapVisualizer(rclcpp::Node& node,
                             rclcpp::Logger& logger,
                             const std::string& mapTopic,
                             const std::string& mapImageTopic)
  : m_logger(&logger)
{
  if (!mapTopic.empty())
  {
    try
    {
      m_mapPublisher =
          node.create_publisher<sensor_msgs::msg::PointCloud2>(mapTopic, rclcpp::SensorDataQoS());
      if (m_logger)
        RCLCPP_INFO(*m_logger, "Publishing SLAM map point cloud on topic: %s", mapTopic.c_str());
    }
    catch (const std::exception& err)
    {
      if (m_logger)
        RCLCPP_ERROR(*m_logger, "Failed to create map point cloud publisher '%s': %s",
                     mapTopic.c_str(), err.what());
      m_mapPublisher.reset();
    }
  }

  if (!mapImageTopic.empty())
  {
    try
    {
      m_mapImagePublisher = std::make_unique<image_transport::Publisher>(
          image_transport::create_publisher(&node, mapImageTopic));
      m_mapImageRenderer = std::make_unique<MapImageRenderer>();
      if (m_logger)
        RCLCPP_INFO(*m_logger, "Publishing SLAM map image on topic: %s", mapImageTopic.c_str());
    }
    catch (const std::exception& err)
    {
      if (m_logger)
        RCLCPP_ERROR(*m_logger, "Failed to create map image publisher '%s': %s",
                     mapImageTopic.c_str(), err.what());
      m_mapImagePublisher.reset();
      m_mapImageRenderer.reset();
    }
  }
}

MapVisualizer::~MapVisualizer() = default;

void MapVisualizer::Publish(const std_msgs::msg::Header& header,
                            const std::vector<ORB_SLAM3::MapPoint*>& trackedMapPoints,
                            const Eigen::Vector3f& cameraPosition,
                            const Eigen::Quaternionf& cameraOrientation)
{
  const bool publishPointCloud = static_cast<bool>(m_mapPublisher);
  const bool publishMapImage = m_mapImagePublisher && m_mapImagePublisher->getNumSubscribers() > 0;

  if (!publishPointCloud && !publishMapImage)
    return;

  std::unordered_set<const ORB_SLAM3::MapPoint*> trackedPointSet;
  trackedPointSet.reserve(trackedMapPoints.size());

  for (const ORB_SLAM3::MapPoint* mapPoint : trackedMapPoints)
  {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    if (!ExtractMapPointPosition(mapPoint, position))
      continue;

    trackedPointSet.insert(mapPoint);
    m_mapPointPositions[mapPoint] = position;
  }

  for (auto it = m_mapPointPositions.begin(); it != m_mapPointPositions.end();)
  {
    const ORB_SLAM3::MapPoint* mapPoint = it->first;
    if (mapPoint == nullptr || const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->isBad())
    {
      it = m_mapPointPositions.erase(it);
      continue;
    }

    Eigen::Vector3f& storedPosition = it->second;
    if (!ExtractMapPointPosition(mapPoint, storedPosition))
    {
      it = m_mapPointPositions.erase(it);
      continue;
    }

    ++it;
  }

  const bool cameraValid = std::isfinite(cameraPosition.x()) && std::isfinite(cameraPosition.y()) &&
                           std::isfinite(cameraPosition.z());

  std::vector<MapPointRenderInfo> renderPoints;
  renderPoints.reserve(m_mapPointPositions.size());

  for (const auto& entry : m_mapPointPositions)
  {
    const Eigen::Vector3f& position = entry.second;
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
        !std::isfinite(position.z()))
      continue;

    const bool isTracked = trackedPointSet.find(entry.first) != trackedPointSet.end();

    renderPoints.push_back({position, isTracked});
  }

  struct VoxelKey
  {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey&) const = default;
  };

  struct VoxelKeyHasher
  {
    std::size_t operator()(const VoxelKey& key) const noexcept
    {
      constexpr std::size_t PRIME1 = 73856093U;
      constexpr std::size_t PRIME2 = 19349663U;
      constexpr std::size_t PRIME3 = 83492791U;
      return static_cast<std::size_t>(key.x) * PRIME1 ^ static_cast<std::size_t>(key.y) * PRIME2 ^
             static_cast<std::size_t>(key.z) * PRIME3;
    }
  };

  struct VoxelAggregate
  {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    std::size_t count = 0;
    std::size_t trackedCount = 0;
  };

  std::unordered_map<VoxelKey, VoxelAggregate, VoxelKeyHasher> voxelizedPoints;
  voxelizedPoints.reserve(renderPoints.size());

  constexpr float VOXEL_SIZE_METERS = 0.08F;
  const float invVoxelSize = 1.0F / VOXEL_SIZE_METERS;

  const auto voxelize = [&](const Eigen::Vector3f& position)
  {
    return VoxelKey{static_cast<int>(std::floor(position.x() * invVoxelSize)),
                    static_cast<int>(std::floor(position.y() * invVoxelSize)),
                    static_cast<int>(std::floor(position.z() * invVoxelSize))};
  };

  for (const MapPointRenderInfo& renderPoint : renderPoints)
  {
    const VoxelKey key = voxelize(renderPoint.position);
    VoxelAggregate& aggregate = voxelizedPoints[key];
    aggregate.sum += renderPoint.position;
    ++aggregate.count;
    if (renderPoint.tracked)
      ++aggregate.trackedCount;
  }

  std::vector<MapPointRenderInfo> aggregatedPoints;
  aggregatedPoints.reserve(voxelizedPoints.size());
  for (const auto& entry : voxelizedPoints)
  {
    const VoxelAggregate& aggregate = entry.second;
    if (aggregate.count == 0)
      continue;

    const Eigen::Vector3f averaged = aggregate.sum / static_cast<float>(aggregate.count);
    aggregatedPoints.push_back({averaged, aggregate.trackedCount > 0});
  }

  const std::vector<MapPointRenderInfo>& pointsToRender =
      aggregatedPoints.empty() ? renderPoints : aggregatedPoints;

  std::size_t validMapPointCount = pointsToRender.size();
  float maxDistance = 0.0F;
  float minDistance = std::numeric_limits<float>::max();
  if (cameraValid)
  {
    for (const auto& renderPoint : pointsToRender)
    {
      const float distance = (renderPoint.position - cameraPosition).norm();
      if (!std::isfinite(distance))
        continue;

      maxDistance = std::max(maxDistance, distance);
      minDistance = std::min(minDistance, distance);
    }
    if (!(minDistance < maxDistance))
      minDistance = 0.0F;
  }

  float minHeight = std::numeric_limits<float>::max();
  float maxHeight = std::numeric_limits<float>::lowest();
  for (const auto& renderPoint : pointsToRender)
  {
    minHeight = std::min(minHeight, renderPoint.position.y());
    maxHeight = std::max(maxHeight, renderPoint.position.y());
  }
  if (!(minHeight < maxHeight))
  {
    minHeight = -1.0F;
    maxHeight = 1.0F;
  }

  bool arrowValid = false;
  Eigen::Vector3f arrowTip = Eigen::Vector3f::Zero();
  if (cameraValid)
  {
    Eigen::Vector3f forwardVector = cameraOrientation * Eigen::Vector3f::UnitZ();
    const bool forwardFinite = std::isfinite(forwardVector.x()) &&
                               std::isfinite(forwardVector.y()) && std::isfinite(forwardVector.z());
    if (forwardFinite)
    {
      const float arrowScale = std::max(0.25F, 0.1F * maxDistance);
      const float forwardNorm = forwardVector.norm();
      if (forwardNorm > std::numeric_limits<float>::epsilon())
      {
        arrowTip = cameraPosition + forwardVector / forwardNorm * arrowScale;
        if (std::isfinite(arrowTip.x()) && std::isfinite(arrowTip.y()) &&
            std::isfinite(arrowTip.z()))
          arrowValid = true;
      }
    }
  }

  if (publishPointCloud)
  {
    if (auto pointCloud = m_pointCloudBuilder.BuildPointCloud(
            header, pointsToRender, validMapPointCount, cameraValid, cameraPosition, arrowValid,
            arrowTip, minDistance, maxDistance, minHeight, maxHeight))
    {
      m_mapPublisher->publish(*pointCloud);
    }
  }

  if (publishMapImage)
    PublishMapImage(header, pointsToRender, cameraPosition, cameraOrientation, maxDistance);
}

void MapVisualizer::Reset()
{
  m_mapPointPositions.clear();
}

void MapVisualizer::PublishMapImage(const std_msgs::msg::Header& header,
                                    const std::vector<MapPointRenderInfo>& renderPoints,
                                    const Eigen::Vector3f& cameraPosition,
                                    const Eigen::Quaternionf& cameraOrientation,
                                    float maxDistance)
{
  if (!m_mapImagePublisher || m_mapImagePublisher->getNumSubscribers() == 0)
    return;

  if (!m_mapImageRenderer)
    return;

  if (auto image = m_mapImageRenderer->RenderImage(header, renderPoints, cameraPosition,
                                                   cameraOrientation, maxDistance))
  {
    m_mapImagePublisher->publish(image);
  }
}
