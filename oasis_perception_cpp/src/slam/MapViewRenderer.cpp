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
#include <limits>
#include <vector>

#include <MapPoint.h>

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

  const Eigen::Matrix3f rotation = Tcw.rotationMatrix();
  const Eigen::Vector3f translation = Tcw.translation();

  std::vector<ProjectedPoint> projectedPoints;
  projectedPoints.reserve(mapPoints.size());

  float minDepth = std::numeric_limits<float>::infinity();
  float maxDepth = 0.0f;

  for (const auto* mapPoint : mapPoints)
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
    maxDepth = std::max(maxDepth, depth);
  }

  if (projectedPoints.empty() || !std::isfinite(minDepth))
    return false;

  const float depthRange = std::max(maxDepth - minDepth, 1e-3f);
  for (const ProjectedPoint& point : projectedPoints)
  {
    const int index = point.y * m_width + point.x;
    if (index < 0 || index >= static_cast<int>(m_depthBuffer.size()))
      continue;

    if (point.depth >= m_depthBuffer[index])
      continue;

    m_depthBuffer[index] = point.depth;
    const float normalizedDepth = (point.depth - minDepth) / depthRange;
    outputImage.at<cv::Vec3b>(point.y, point.x) = m_paletteSampler.Sample(normalizedDepth);
  }

  return true;
}

void MapViewRenderer::ResizeBuffers()
{
  const std::size_t bufferSize = static_cast<std::size_t>(m_width) * static_cast<std::size_t>(m_height);
  if (bufferSize == m_depthBuffer.size())
    return;

  m_depthBuffer.assign(bufferSize, std::numeric_limits<float>::infinity());
}
