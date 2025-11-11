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

using namespace OASIS;
using namespace SLAM;

void MapViewRenderer::Initialize(const CameraModel& model)
{
  m_cameraModel = model;

  // Set image parameters
  SetImageSize(m_cameraModel.width, m_cameraModel.height);
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

bool MapViewRenderer::Render(const Sophus::SE3f& cameraFromWorldTransform,
                             const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
                             cv::Mat& outputImage)
{
  if (!CanRender())
    return false;

  PrepareRender(outputImage);

  const std::vector<ProjectedPoint> projectedPoints =
      ProjectMapPoints(cameraFromWorldTransform, mapPoints);
  if (projectedPoints.empty())
    return false;

  const std::vector<float> normalizedDepths = ComputeNormalizedDepths(projectedPoints);
  const float averageFocal = 0.5f * (m_cameraModel.fx + m_cameraModel.fy);

  for (std::size_t idx = 0; idx < projectedPoints.size(); ++idx)
    RenderProjectedPoint(projectedPoints[idx], normalizedDepths[idx], averageFocal);

  ComposeOutputImage(outputImage);

  return true;
}

bool MapViewRenderer::CanRender() const
{
  return m_width > 0 && m_height > 0 && m_cameraModel.width > 0 && m_cameraModel.height > 0;
}

void MapViewRenderer::PrepareRender(cv::Mat& outputImage)
{
  ResizeBuffers();

  outputImage.create(m_height, m_width, CV_8UC3);
  outputImage.setTo(cv::Scalar(0, 0, 0));

  std::fill(m_depthBuffer.begin(), m_depthBuffer.end(), std::numeric_limits<float>::infinity());
  std::fill(m_colorBuffer.begin(), m_colorBuffer.end(), cv::Vec3f(0.0f, 0.0f, 0.0f));
  std::fill(m_weightBuffer.begin(), m_weightBuffer.end(), 0.0f);
}

std::vector<MapViewRenderer::ProjectedPoint> MapViewRenderer::ProjectMapPoints(
    const Sophus::SE3f& cameraFromWorldTransform,
    const std::vector<ORB_SLAM3::MapPoint*>& mapPoints) const
{
  std::vector<ProjectedPoint> projectedPoints;
  projectedPoints.reserve(mapPoints.size());

  const Eigen::Matrix3f rotation = cameraFromWorldTransform.rotationMatrix();
  const Eigen::Vector3f translation = cameraFromWorldTransform.translation();

  float minDepth = std::numeric_limits<float>::infinity();

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

    projectedPoints.push_back({pixelX, pixelY, depth});
    minDepth = std::min(minDepth, depth);
  }

  if (!std::isfinite(minDepth))
    projectedPoints.clear();

  return projectedPoints;
}

std::vector<float> MapViewRenderer::ComputeNormalizedDepths(
    const std::vector<ProjectedPoint>& projectedPoints) const
{
  std::vector<float> normalizedDepths(projectedPoints.size(), 0.0f);
  if (projectedPoints.size() <= 1)
  {
    if (projectedPoints.size() == 1)
      normalizedDepths[0] = 0.5f;
    return normalizedDepths;
  }

  struct DepthSample
  {
    float depth;
    std::size_t index;
  };

  std::vector<DepthSample> depthSamples;
  depthSamples.reserve(projectedPoints.size());

  for (std::size_t i = 0; i < projectedPoints.size(); ++i)
    depthSamples.push_back({projectedPoints[i].depth, i});

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
    const float normalizedValue = averageRank / static_cast<float>(depthSamples.size() - 1);

    for (std::size_t k = i; k < j; ++k)
      normalizedDepths[depthSamples[k].index] = normalizedValue;

    i = j;
  }

  return normalizedDepths;
}

void MapViewRenderer::RenderProjectedPoint(const ProjectedPoint& point,
                                           float normalizedDepth,
                                           float averageFocal)
{
  const cv::Vec3b sampledColor = m_paletteSampler.Sample(std::clamp(normalizedDepth, 0.0f, 1.0f));
  const cv::Vec3f colorVec(static_cast<float>(sampledColor[0]), static_cast<float>(sampledColor[1]),
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

void MapViewRenderer::ComposeOutputImage(cv::Mat& outputImage) const
{
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
}
