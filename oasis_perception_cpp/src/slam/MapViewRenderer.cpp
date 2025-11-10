/*
 * Copyright (C) 2025 Garrett Brown
 * This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 * SPDX-License-Identifier: Apache-2.0
 * See the file LICENSE.txt for more information.
 */

#include "MapViewRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <MapPoint.h>
#include <opencv2/imgproc.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
struct ProjectedPoint
{
  int x{0};
  int y{0};
  float depth{0.0f};
};

} // namespace

void MapViewRenderer::SetCameraModel(const CameraModel& model)
{
  m_cameraModel = model;
}

void MapViewRenderer::SetImageSize(int width, int height)
{
  if (width <= 0 || height <= 0)
    return;

  if (width == m_width && height == m_height)
    return;

  m_width = width;
  m_height = height;
  ResizeBuffers();
}

void MapViewRenderer::EnableRoiMask(bool enable)
{
  if (m_roiMaskEnabled == enable)
    return;

  m_roiMaskEnabled = enable;
  m_roiMaskDirty = true;
}

void MapViewRenderer::SetRoiPolygon(const std::vector<cv::Point>& polygon)
{
  m_roiPolygon = polygon;
  m_roiMaskDirty = true;
}

bool MapViewRenderer::Render(const Sophus::SE3f& Tcw,
                             const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
                             cv::Mat& outputImage)
{
  if (m_width <= 0 || m_height <= 0 || m_cameraModel.width <= 0 || m_cameraModel.height <= 0)
    return false;

  ResizeBuffers();

  outputImage.create(m_height, m_width, CV_8UC3);
  outputImage.setTo(cv::Scalar(0, 0, 0));

  std::fill(m_depthBuffer.begin(), m_depthBuffer.end(), std::numeric_limits<float>::infinity());
  std::fill(m_colorBuffer.begin(), m_colorBuffer.end(), cv::Vec3f(0.0f, 0.0f, 0.0f));
  std::fill(m_weightBuffer.begin(), m_weightBuffer.end(), 0.0f);

  const Eigen::Matrix3f rotation = Tcw.rotationMatrix();
  const Eigen::Vector3f translation = Tcw.translation();

  if (m_roiMaskEnabled)
  {
    UpdateRoiMask();
  }
  else if (!m_roiMask.empty())
  {
    m_roiMask.release();
  }

  std::vector<ProjectedPoint> projectedPoints;
  projectedPoints.reserve(mapPoints.size());

  for (auto* mapPoint : mapPoints)
  {
    if (mapPoint == nullptr || mapPoint->isBad())
      continue;

    const Eigen::Vector3f worldPos = mapPoint->GetWorldPos();
    const Eigen::Vector3f cameraPos = rotation * worldPos + translation;
    const float depth = cameraPos.z();
    if (!std::isfinite(depth) || depth <= 0.0f)
      continue;

    const float invDepth = 1.0f / depth;
    const float u = m_cameraModel.fx * cameraPos.x() * invDepth + m_cameraModel.cx;
    const float v = m_cameraModel.fy * cameraPos.y() * invDepth + m_cameraModel.cy;

    const int pixelX = static_cast<int>(std::lround(u));
    const int pixelY = static_cast<int>(std::lround(v));
    if (pixelX < 0 || pixelX >= m_width || pixelY < 0 || pixelY >= m_height)
      continue;

    if (m_roiMaskEnabled && !m_roiMask.empty() && m_roiMask.at<std::uint8_t>(pixelY, pixelX) == 0)
      continue;

    projectedPoints.push_back({pixelX, pixelY, depth});
  }

  if (projectedPoints.empty())
    return false;

  const int cellSize = 16;
  const int gridWidth = (m_width + cellSize - 1) / cellSize;
  const int gridHeight = (m_height + cellSize - 1) / cellSize;
  std::vector<std::vector<std::size_t>> grid(static_cast<std::size_t>(gridWidth) *
                                             static_cast<std::size_t>(gridHeight));

  for (std::size_t idx = 0; idx < projectedPoints.size(); ++idx)
  {
    const ProjectedPoint& point = projectedPoints[idx];
    const int cellX = std::clamp(point.x / cellSize, 0, gridWidth - 1);
    const int cellY = std::clamp(point.y / cellSize, 0, gridHeight - 1);
    grid[static_cast<std::size_t>(cellY) * static_cast<std::size_t>(gridWidth) +
         static_cast<std::size_t>(cellX)]
        .push_back(idx);
  }

  std::vector<bool> keep(projectedPoints.size(), true);
  const float pixelRadius = 12.0f;
  const float pixelRadiusSquared = pixelRadius * pixelRadius;
  const float absoluteDepthThreshold = 0.25f;
  const float relativeDepthThreshold = 0.12f;
  const int neighborRange = 1;

  for (std::size_t idx = 0; idx < projectedPoints.size(); ++idx)
  {
    const ProjectedPoint& point = projectedPoints[idx];
    int requiredSupport = 0;
    if (point.depth > 30.0f)
      requiredSupport = 3;
    else if (point.depth > 12.0f)
      requiredSupport = 2;
    else if (point.depth > 4.0f)
      requiredSupport = 1;

    if (requiredSupport == 0)
      continue;

    const int cellX = std::clamp(point.x / cellSize, 0, gridWidth - 1);
    const int cellY = std::clamp(point.y / cellSize, 0, gridHeight - 1);
    int support = 0;

    for (int offsetY = -neighborRange; offsetY <= neighborRange; ++offsetY)
    {
      const int neighborCellY = cellY + offsetY;
      if (neighborCellY < 0 || neighborCellY >= gridHeight)
        continue;

      for (int offsetX = -neighborRange; offsetX <= neighborRange; ++offsetX)
      {
        const int neighborCellX = cellX + offsetX;
        if (neighborCellX < 0 || neighborCellX >= gridWidth)
          continue;

        const std::vector<std::size_t>& cellPoints =
            grid[static_cast<std::size_t>(neighborCellY) * static_cast<std::size_t>(gridWidth) +
                 static_cast<std::size_t>(neighborCellX)];

        for (std::size_t neighborIdx : cellPoints)
        {
          if (neighborIdx == idx)
            continue;

          const ProjectedPoint& neighborPoint = projectedPoints[neighborIdx];
          const float dx = static_cast<float>(neighborPoint.x - point.x);
          const float dy = static_cast<float>(neighborPoint.y - point.y);
          const float distanceSquared = dx * dx + dy * dy;
          if (distanceSquared > pixelRadiusSquared)
            continue;

          const float depthDifference = std::abs(neighborPoint.depth - point.depth);
          const float allowedDepthDifference =
              std::max(absoluteDepthThreshold,
                       relativeDepthThreshold * std::min(neighborPoint.depth, point.depth));
          if (depthDifference > allowedDepthDifference)
            continue;

          ++support;
          if (support >= requiredSupport)
            break;
        }

        if (support >= requiredSupport)
          break;
      }

      if (support >= requiredSupport)
        break;
    }

    if (support < requiredSupport)
      keep[idx] = false;
  }

  const std::size_t keptCount = std::count(keep.begin(), keep.end(), true);
  if (keptCount == 0)
    return false;

  struct DepthSample
  {
    float depth;
    std::size_t index;
  };

  std::vector<DepthSample> depthSamples;
  depthSamples.reserve(keptCount);
  for (std::size_t idx = 0; idx < projectedPoints.size(); ++idx)
  {
    if (!keep[idx])
      continue;

    depthSamples.push_back({projectedPoints[idx].depth, idx});
  }

  std::vector<float> normalizedDepths(projectedPoints.size(), 0.0f);
  if (depthSamples.size() >= 2)
  {
    std::sort(depthSamples.begin(), depthSamples.end(),
              [](const DepthSample& lhs, const DepthSample& rhs) { return lhs.depth < rhs.depth; });

    std::size_t i = 0;
    while (i < depthSamples.size())
    {
      const float currentDepth = depthSamples[i].depth;
      std::size_t j = i + 1;
      while (j < depthSamples.size() && depthSamples[j].depth == currentDepth)
        ++j;

      const float rankStart = static_cast<float>(i);
      const float rankEnd = static_cast<float>(j - 1);
      const float averageRank = 0.5f * (rankStart + rankEnd);
      const float normalizedValue = (depthSamples.size() > 1)
                                        ? averageRank / static_cast<float>(depthSamples.size() - 1)
                                        : 0.0f;

      for (std::size_t k = i; k < j; ++k)
        normalizedDepths[depthSamples[k].index] = normalizedValue;

      i = j;
    }
  }
  else if (depthSamples.size() == 1)
  {
    normalizedDepths[depthSamples[0].index] = 0.5f;
  }

  const float averageFocal = 0.5f * (m_cameraModel.fx + m_cameraModel.fy);

  for (std::size_t idx = 0; idx < projectedPoints.size(); ++idx)
  {
    if (!keep[idx])
      continue;

    const ProjectedPoint& point = projectedPoints[idx];
    const float normalizedDepth = normalizedDepths[idx];
    const cv::Vec3b sampledColor = m_paletteSampler.Sample(std::clamp(normalizedDepth, 0.0f, 1.0f));
    const cv::Vec3f colorVec(static_cast<float>(sampledColor[0]),
                             static_cast<float>(sampledColor[1]),
                             static_cast<float>(sampledColor[2]));

    const float pointDepth = std::max(point.depth, 1e-3f);
    const float radius = std::clamp(averageFocal / (pointDepth * 120.0f), 1.5f, 6.0f);
    const int radiusInt = static_cast<int>(std::ceil(radius));
    const float radiusSquared = radius * radius;
    const float sigma = std::max(radius * 0.5f, 0.75f);
    const float invTwoSigmaSquared = 1.0f / (2.0f * sigma * sigma);
    const float depthTolerance = std::max(0.02f * point.depth, 0.02f);

    for (int dy = -radiusInt; dy <= radiusInt; ++dy)
    {
      const int pixelY = point.y + dy;
      if (pixelY < 0 || pixelY >= m_height)
        continue;

      for (int dx = -radiusInt; dx <= radiusInt; ++dx)
      {
        const int pixelX = point.x + dx;
        if (pixelX < 0 || pixelX >= m_width)
          continue;

        const float distanceSquared = static_cast<float>(dx * dx + dy * dy);
        if (distanceSquared > radiusSquared)
          continue;

        const int index = pixelY * m_width + pixelX;
        if (index < 0 || index >= static_cast<int>(m_depthBuffer.size()))
          continue;

        if (point.depth > m_depthBuffer[index] + depthTolerance)
          continue;

        const float weight = std::exp(-distanceSquared * invTwoSigmaSquared);

        if (point.depth < m_depthBuffer[index] - depthTolerance)
        {
          m_depthBuffer[index] = point.depth;
          m_colorBuffer[index] = weight * colorVec;
          m_weightBuffer[index] = weight;
        }
        else
        {
          m_depthBuffer[index] = std::min(m_depthBuffer[index], point.depth);
          m_colorBuffer[index] += weight * colorVec;
          m_weightBuffer[index] += weight;
        }
      }
    }
  }

  for (int y = 0; y < m_height; ++y)
  {
    for (int x = 0; x < m_width; ++x)
    {
      const int index = y * m_width + x;
      const float weight = m_weightBuffer[index];
      if (weight <= 0.0f)
        continue;

      const cv::Vec3f color = m_colorBuffer[index] / weight;
      outputImage.at<cv::Vec3b>(y, x) =
          cv::Vec3b(static_cast<std::uint8_t>(std::clamp(color[0], 0.0f, 255.0f)),
                    static_cast<std::uint8_t>(std::clamp(color[1], 0.0f, 255.0f)),
                    static_cast<std::uint8_t>(std::clamp(color[2], 0.0f, 255.0f)));
    }
  }

  return true;
}

void MapViewRenderer::UpdateRoiMask()
{
  if (!m_roiMaskEnabled)
  {
    m_roiMask.release();
    return;
  }

  if (m_width <= 0 || m_height <= 0)
  {
    m_roiMask.release();
    return;
  }

  if (!m_roiMaskDirty && !m_roiMask.empty() && m_roiMask.cols == m_width &&
      m_roiMask.rows == m_height)
    return;

  cv::Mat mask(m_height, m_width, CV_8UC1, cv::Scalar(255));

  std::vector<cv::Point> polygon = m_roiPolygon;
  if (polygon.empty())
  {
    const int cutoff =
        std::max(1, static_cast<int>(std::round(0.28f * static_cast<float>(m_width))));
    const int maxX = std::max(0, m_width - 1);
    const int maxY = std::max(0, m_height - 1);
    polygon = {cv::Point(0, 0), cv::Point(std::min(cutoff, maxX), 0),
               cv::Point(std::min(cutoff, maxX), maxY), cv::Point(0, maxY)};
  }
  else
  {
    for (auto& point : polygon)
    {
      point.x = std::clamp(point.x, 0, std::max(0, m_width - 1));
      point.y = std::clamp(point.y, 0, std::max(0, m_height - 1));
    }
  }

  if (polygon.size() >= 3)
  {
    std::vector<std::vector<cv::Point>> polygons = {polygon};
    cv::fillPoly(mask, polygons, cv::Scalar(0));
  }

  m_roiMask = std::move(mask);
  m_roiMaskDirty = false;
}

void MapViewRenderer::ResizeBuffers()
{
  const std::size_t bufferSize =
      static_cast<std::size_t>(m_width) * static_cast<std::size_t>(m_height);
  if (bufferSize == m_depthBuffer.size())
    return;

  m_depthBuffer.assign(bufferSize, std::numeric_limits<float>::infinity());
  m_colorBuffer.assign(bufferSize, cv::Vec3f(0.0f, 0.0f, 0.0f));
  m_weightBuffer.assign(bufferSize, 0.0f);
  m_roiMaskDirty = true;
}
