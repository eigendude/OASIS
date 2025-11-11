/*
 * Copyright (C) 2025 Garrett Brown
 * This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 * SPDX-License-Identifier: Apache-2.0
 * See the file LICENSE.txt for more information.
 */

#include "MapViewRenderer.h"

#include "ViridisPaletteSampler.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <MapPoint.h>

using namespace OASIS;
using namespace SLAM;

void MapViewRenderer::Initialize(const CameraModel& cameraModel)
{
  m_cameraModel = cameraModel;

  ResizeBuffers(m_cameraModel, m_imageBuffers);
}

bool MapViewRenderer::Render(const Eigen::Isometry3f& cameraFromWorldTransform,
                             const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
                             cv::Mat& outputImage)
{
  if (!CanRender(m_cameraModel))
    return false;

  PrepareRender(m_cameraModel, m_imageBuffers, outputImage);

  ProjectMapPoints(m_cameraModel, cameraFromWorldTransform, mapPoints, m_projectedPoints);
  if (m_projectedPoints.empty())
    return false;

  ComputeNormalizedDepths(m_projectedPoints, m_depthBuffer, m_normalizedDepths);

  const float averageFocal = 0.5f * (m_cameraModel.fx + m_cameraModel.fy);

  for (std::size_t idx = 0; idx < m_projectedPoints.size(); ++idx)
    RenderProjectedPoint(m_cameraModel, m_projectedPoints[idx], m_normalizedDepths[idx],
                         averageFocal, m_imageBuffers);

  ComposeOutputImage(m_cameraModel, m_imageBuffers, outputImage);

  return true;
}

void MapViewRenderer::ResizeBuffers(const CameraModel& cameraModel, ImageBuffers& imageBuffers)
{
  const std::size_t bufferSize =
      static_cast<std::size_t>(cameraModel.width) * static_cast<std::size_t>(cameraModel.height);
  if (bufferSize == imageBuffers.depthBuffer.size())
    return;

  imageBuffers.depthBuffer.assign(bufferSize, std::numeric_limits<float>::infinity());
  imageBuffers.colorBuffer.assign(bufferSize, cv::Vec3f(0.0f, 0.0f, 0.0f));
  imageBuffers.weightBuffer.assign(bufferSize, 0.0f);
}

bool MapViewRenderer::CanRender(const CameraModel& cameraModel)
{
  return cameraModel.width > 0 && cameraModel.height > 0;
}

void MapViewRenderer::PrepareRender(const CameraModel& cameraModel,
                                    ImageBuffers& imageBuffers,
                                    cv::Mat& outputImage)
{
  std::fill(imageBuffers.depthBuffer.begin(), imageBuffers.depthBuffer.end(),
            std::numeric_limits<float>::infinity());
  std::fill(imageBuffers.colorBuffer.begin(), imageBuffers.colorBuffer.end(),
            cv::Vec3f(0.0f, 0.0f, 0.0f));
  std::fill(imageBuffers.weightBuffer.begin(), imageBuffers.weightBuffer.end(), 0.0f);

  outputImage.create(static_cast<int>(cameraModel.height), static_cast<int>(cameraModel.width),
                     CV_8UC3);
  outputImage.setTo(cv::Scalar(0, 0, 0));
}

void MapViewRenderer::ProjectMapPoints(const CameraModel& cameraModel,
                                       const Eigen::Isometry3f& cameraFromWorldTransform,
                                       const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
                                       std::vector<ProjectedPoint>& projectedPoints)
{
  projectedPoints.clear();
  projectedPoints.reserve(mapPoints.size());

  const Eigen::Matrix3f rotation = cameraFromWorldTransform.linear();
  const Eigen::Vector3f translation = cameraFromWorldTransform.translation();

  float minDepth = std::numeric_limits<float>::infinity();

  for (const ORB_SLAM3::MapPoint* mapPoint : mapPoints)
  {
    if (mapPoint == nullptr || const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->isBad())
      continue;

    const Eigen::Vector3f worldPos = const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->GetWorldPos();
    const Eigen::Vector3f cameraPos = rotation * worldPos + translation;
    const float depth = cameraPos.z();
    if (!std::isfinite(depth) || depth <= 0.0f)
      continue;

    const float invDepth = 1.0f / depth;
    const float u = cameraModel.fx * cameraPos.x() * invDepth + cameraModel.cx;
    const float v = cameraModel.fy * cameraPos.y() * invDepth + cameraModel.cy;

    const int pixelX = static_cast<int>(std::lround(u));
    const int pixelY = static_cast<int>(std::lround(v));
    if (pixelX < 0 || pixelX >= static_cast<int>(cameraModel.width) || pixelY < 0 ||
        pixelY >= static_cast<int>(cameraModel.height))
      continue;

    projectedPoints.push_back({pixelX, pixelY, depth});
    minDepth = std::min(minDepth, depth);
  }
}

void MapViewRenderer::ComputeNormalizedDepths(const std::vector<ProjectedPoint>& projectedPoints,
                                              std::vector<DepthSample>& depthBuffer,
                                              std::vector<float>& normalizedDepths)
{
  normalizedDepths.resize(projectedPoints.size());
  if (projectedPoints.size() <= 1)
  {
    if (projectedPoints.size() == 1)
      normalizedDepths[0] = 0.5f;
    return;
  }

  depthBuffer.resize(projectedPoints.size());

  for (std::size_t i = 0; i < projectedPoints.size(); ++i)
    depthBuffer[i] = {projectedPoints[i].depth, i};

  std::sort(depthBuffer.begin(), depthBuffer.end(),
            [](const DepthSample& lhs, const DepthSample& rhs) { return lhs.depth < rhs.depth; });

  std::size_t i = 0;
  while (i < depthBuffer.size())
  {
    const float currentDepth = depthBuffer[i].depth;
    std::size_t j = i + 1;
    while (j < depthBuffer.size() && depthBuffer[j].depth == currentDepth)
      ++j;

    const float rankStart = static_cast<float>(i);
    const float rankEnd = static_cast<float>(j - 1);
    const float averageRank = 0.5f * (rankStart + rankEnd);
    const float normalizedValue = averageRank / static_cast<float>(depthBuffer.size() - 1);

    for (std::size_t k = i; k < j; ++k)
      normalizedDepths[depthBuffer[k].index] = normalizedValue;

    i = j;
  }
}

void MapViewRenderer::RenderProjectedPoint(const CameraModel& cameraModel,
                                           const ProjectedPoint& point,
                                           float normalizedDepth,
                                           float averageFocal,
                                           ImageBuffers& imageBuffers)
{
  ViridisPaletteSampler paletteSampler;

  const cv::Vec3b sampledColor = paletteSampler.Sample(std::clamp(normalizedDepth, 0.0f, 1.0f));
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
    if (pixelY < 0 || pixelY >= static_cast<int>(cameraModel.height))
      continue;

    for (int dx = -radiusInt; dx <= radiusInt; ++dx)
    {
      const int pixelX = point.x + dx;
      if (pixelX < 0 || pixelX >= static_cast<int>(cameraModel.width))
        continue;

      const float distanceSquared = static_cast<float>(dx * dx + dy * dy);
      if (distanceSquared > radiusSquared)
        continue;

      const int index = pixelY * static_cast<int>(cameraModel.width) + pixelX;
      if (index < 0 || index >= static_cast<int>(imageBuffers.depthBuffer.size()))
        continue;

      if (point.depth > imageBuffers.depthBuffer[index] + depthTolerance)
        continue;

      const float weight = std::exp(-distanceSquared * invTwoSigmaSquared);

      if (point.depth < imageBuffers.depthBuffer[index] - depthTolerance)
      {
        imageBuffers.depthBuffer[index] = point.depth;
        imageBuffers.colorBuffer[index] = weight * colorVec;
        imageBuffers.weightBuffer[index] = weight;
      }
      else
      {
        imageBuffers.depthBuffer[index] = std::min(imageBuffers.depthBuffer[index], point.depth);
        imageBuffers.colorBuffer[index] += weight * colorVec;
        imageBuffers.weightBuffer[index] += weight;
      }
    }
  }
}

void MapViewRenderer::ComposeOutputImage(const CameraModel& cameraModel,
                                         const ImageBuffers& imageBuffers,
                                         cv::Mat& outputImage)
{
  for (unsigned int y = 0; y < cameraModel.height; ++y)
  {
    for (unsigned int x = 0; x < cameraModel.width; ++x)
    {
      const unsigned int index = y * cameraModel.width + x;

      const float weight = imageBuffers.weightBuffer[index];
      if (weight <= 0.0f)
        continue;

      const cv::Vec3f color = imageBuffers.colorBuffer[index] / weight;

      outputImage.at<cv::Vec3b>(y, x) =
          cv::Vec3b(static_cast<std::uint8_t>(std::clamp(color[0], 0.0f, 255.0f)),
                    static_cast<std::uint8_t>(std::clamp(color[1], 0.0f, 255.0f)),
                    static_cast<std::uint8_t>(std::clamp(color[2], 0.0f, 255.0f)));
    }
  }
}
