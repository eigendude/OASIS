/*
 * Copyright (C) 2025 Garrett Brown
 * This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 * SPDX-License-Identifier: Apache-2.0
 * See the file LICENSE.txt for more information.
 */

#include "MapImageRenderer.h"

#include "ViridisPaletteSampler.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 480;
constexpr float FIELD_OF_VIEW_DEGREES = 90.0F;
constexpr float PI = 3.14159265358979323846F;
constexpr float NEAR_PLANE = 0.05F;

float Clamp01(float value)
{
  return std::clamp(value, 0.0F, 1.0F);
}
} // namespace

MapImageRenderer::MapImageRenderer() = default;

sensor_msgs::msg::Image::SharedPtr MapImageRenderer::RenderImage(
    const std_msgs::msg::Header& header,
    const std::vector<MapPointRenderInfo>& renderPoints,
    const Eigen::Vector3f& cameraPosition,
    const Eigen::Quaternionf& cameraOrientation,
    float maxDistance)
{
  if (!cameraPosition.allFinite())
    return nullptr;

  if (!cameraOrientation.coeffs().array().isFinite().all())
    return nullptr;

  Eigen::Quaternionf orientation = cameraOrientation;
  const float orientationNorm = orientation.norm();
  if (!(orientationNorm > std::numeric_limits<float>::epsilon()))
    return nullptr;

  orientation.normalize();

  const float fovRadians = FIELD_OF_VIEW_DEGREES * PI / 180.0F;
  const float halfWidth = static_cast<float>(IMAGE_WIDTH) * 0.5F;
  const float halfHeight = static_cast<float>(IMAGE_HEIGHT) * 0.5F;
  const float focalLength = halfWidth / std::tan(fovRadians * 0.5F);
  const float fx = focalLength;
  const float fy = focalLength;
  const float cx = halfWidth;
  const float cy = halfHeight;

  const float farPlane =
      std::max(NEAR_PLANE + 1e-3F, maxDistance > 0.0F ? maxDistance * 1.25F : 5.0F);

  cv::Mat mapImageBgr(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

  struct ProjectedPoint
  {
    cv::Point pixel;
    float depth = 0.0F;
    float depthFactor = 0.0F;
    float depthNormalized = 0.0F;
    bool tracked = false;
  };

  std::vector<ProjectedPoint> projectedPoints;
  projectedPoints.reserve(renderPoints.size());
  std::size_t visibleCount = 0;
  std::size_t trackedVisibleCount = 0;

  for (const auto& point : renderPoints)
  {
    Eigen::Vector3f relative = point.position - cameraPosition;
    if (!relative.allFinite())
      continue;

    Eigen::Vector3f cameraSpace = orientation.conjugate() * relative;
    if (!cameraSpace.allFinite())
      continue;

    if (cameraSpace.z() <= NEAR_PLANE)
      continue;

    const float invZ = 1.0F / cameraSpace.z();
    const float u = fx * cameraSpace.x() * invZ + cx;
    const float v = fy * cameraSpace.y() * invZ + cy;

    if (u < 0.0F || u >= static_cast<float>(IMAGE_WIDTH) || v < 0.0F ||
        v >= static_cast<float>(IMAGE_HEIGHT))
      continue;

    const float clampedDepth = std::min(farPlane, cameraSpace.z());
    const float depthNormalized = Clamp01((clampedDepth - NEAR_PLANE) / (farPlane - NEAR_PLANE));
    float depthFactor = 1.0F - depthNormalized;
    depthFactor = std::clamp(depthFactor, 0.1F, 1.0F);

    projectedPoints.push_back({
        cv::Point{static_cast<int>(std::round(u)), static_cast<int>(std::round(v))},
        cameraSpace.z(),
        depthFactor,
        depthNormalized,
        point.tracked,
    });

    ++visibleCount;
    if (point.tracked)
      ++trackedVisibleCount;
  }

  std::vector<float> depths;
  depths.reserve(projectedPoints.size());
  for (const auto& p : projectedPoints)
    depths.push_back(p.depth);

  if (!depths.empty())
  {
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 20, depths.end());
    const float depthP05 = depths[depths.size() / 20];

    std::nth_element(depths.begin(), depths.begin() + depths.size() * 19 / 20, depths.end());
    const float depthP95 = depths[depths.size() * 19 / 20];

    m_depthEma.Update(std::min(depthP05, depthP95), std::max(depthP05, depthP95));
    const float zMin = m_depthEma.zMin;
    const float zMax = m_depthEma.zMax;
    const float zSpan = std::max(zMax - zMin, 1e-3F);

    auto depthToViridis = [&](float z) -> Eigen::Vector3f
    {
      float t = Clamp01((z - zMin) / zSpan);

      constexpr float gamma = 0.9f;

      t = std::pow(t, gamma);
      t = 1.0f - t;

      return ViridisPaletteSampler::Sample(t);
    };

    std::stable_sort(projectedPoints.begin(), projectedPoints.end(),
                     [](const ProjectedPoint& a, const ProjectedPoint& b)
                     { return a.depth > b.depth; });

    for (const ProjectedPoint& point : projectedPoints)
    {
      Eigen::Vector3f color = depthToViridis(point.depth);
      color = color.cwiseMax(0.0f).cwiseMin(1.0f);

      const auto to8 = [](float v)
      {
        const long s = std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f);
        return static_cast<int>(std::clamp(s, 0L, 255L));
      };

      cv::Scalar bgr(to8(color.z()), to8(color.y()), to8(color.x()));

      float pxSize = (2.0F * fx) / std::max(point.depth, 1e-3F);
      pxSize *= 0.8F + 0.4F * point.depthFactor;
      pxSize = std::clamp(pxSize, 2.0F, 14.0F);
      const int radius = std::clamp(static_cast<int>(std::round(0.5F * pxSize)), 1, 8);

      cv::circle(mapImageBgr, point.pixel, radius, bgr, cv::FILLED, cv::LINE_AA);

      if (point.tracked)
      {
        const cv::Scalar highlightColor(255, 255, 255);
        const int haloRadius = std::min(radius + 2, 12);
        cv::circle(mapImageBgr, point.pixel, haloRadius, highlightColor, 1, cv::LINE_AA);

        const int armInner = radius + 1;
        const int armOuter = std::max(armInner + 4, 14);
        const int crossThickness = 1;
        cv::line(mapImageBgr, cv::Point(point.pixel.x - armOuter, point.pixel.y),
                 cv::Point(point.pixel.x - armInner, point.pixel.y), highlightColor,
                 crossThickness, cv::LINE_AA);
        cv::line(mapImageBgr, cv::Point(point.pixel.x + armInner, point.pixel.y),
                 cv::Point(point.pixel.x + armOuter, point.pixel.y), highlightColor,
                 crossThickness, cv::LINE_AA);
        cv::line(mapImageBgr, cv::Point(point.pixel.x, point.pixel.y - armOuter),
                 cv::Point(point.pixel.x, point.pixel.y - armInner), highlightColor,
                 crossThickness, cv::LINE_AA);
        cv::line(mapImageBgr, cv::Point(point.pixel.x, point.pixel.y + armInner),
                 cv::Point(point.pixel.x, point.pixel.y + armOuter), highlightColor,
                 crossThickness, cv::LINE_AA);
      }
    }
  }

  cv::line(mapImageBgr, cv::Point(IMAGE_WIDTH / 2, 0), cv::Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT),
           cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
  cv::line(mapImageBgr, cv::Point(0, IMAGE_HEIGHT / 2), cv::Point(IMAGE_WIDTH, IMAGE_HEIGHT / 2),
           cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
  cv::circle(mapImageBgr, cv::Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2), 6, cv::Scalar(80, 80, 80),
             1, cv::LINE_AA);

  std::string overlayText = "Visible: " + std::to_string(visibleCount) +
                            " | Tracked: " + std::to_string(trackedVisibleCount);
  cv::putText(mapImageBgr, overlayText, cv::Point(8, IMAGE_HEIGHT - 12), cv::FONT_HERSHEY_SIMPLEX,
              0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

  ApplyFisheyeEffect(mapImageBgr);

  cv::Mat mapImageRgb;
  cv::cvtColor(mapImageBgr, mapImageRgb, cv::COLOR_BGR2RGB);

  cv_bridge::CvImage cvImage;
  cvImage.header = header;
  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
  cvImage.image = mapImageRgb;

  return cvImage.toImageMsg();
}

void MapImageRenderer::DepthEMA::Update(float newMin, float newMax, float alpha)
{
  if (!init)
  {
    zMin = newMin;
    zMax = newMax;
    init = true;
    return;
  }

  zMin = alpha * newMin + (1.0F - alpha) * zMin;
  zMax = alpha * newMax + (1.0F - alpha) * zMax;

  if (!(zMin < zMax))
    zMax = zMin + 1e-3F;
}

void MapImageRenderer::ApplyFisheyeEffect(cv::Mat& image)
{
  if (image.empty())
    return;

  const cv::Size imageSize = image.size();
  if (m_cachedSize != imageSize)
  {
    m_cachedSize = imageSize;

    m_mapX.create(imageSize, CV_32FC1);
    m_mapY.create(imageSize, CV_32FC1);

    const int width = image.cols;
    const int height = image.rows;
    if (width == 0 || height == 0)
      return;

    const float cx = 0.5F * static_cast<float>(width);
    const float cy = 0.5F * static_cast<float>(height);
    const float invCx = 1.0F / cx;
    const float invCy = 1.0F / cy;

    constexpr float k1 = -0.35F;
    constexpr float k2 = 0.12F;

    for (int y = 0; y < height; ++y)
    {
      float* mapXRow = m_mapX.ptr<float>(y);
      float* mapYRow = m_mapY.ptr<float>(y);

      const float ny = (static_cast<float>(y) - cy) * invCy;
      for (int x = 0; x < width; ++x)
      {
        const float nx = (static_cast<float>(x) - cx) * invCx;
        const float r2 = nx * nx + ny * ny;
        const float r4 = r2 * r2;
        const float distortion = 1.0F + k1 * r2 + k2 * r4;

        const float srcX = cx + nx * distortion * cx;
        const float srcY = cy + ny * distortion * cy;

        if (srcX >= 0.0F && srcX < static_cast<float>(width) && srcY >= 0.0F &&
            srcY < static_cast<float>(height))
        {
          mapXRow[x] = srcX;
          mapYRow[x] = srcY;
        }
        else
        {
          mapXRow[x] = -1.0F;
          mapYRow[x] = -1.0F;
        }
      }
    }
  }

  cv::Mat distorted;
  cv::remap(image, distorted, m_mapX, m_mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
            cv::Scalar(0, 0, 0));
  distorted.copyTo(image);
}

