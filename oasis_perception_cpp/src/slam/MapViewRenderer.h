/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "CameraModel.h"

#include <array>
#include <cstddef>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace ORB_SLAM3
{
class MapPoint;
} // namespace ORB_SLAM3

namespace OASIS
{
namespace SLAM
{

class MapViewRenderer
{
public:
  MapViewRenderer() = default;

  void Initialize(const CameraModel& cameraModel);

  bool Render(const Eigen::Isometry3f& cameraFromWorldTransform,
              const std::vector<Eigen::Vector3f>& worldPoints,
              cv::Mat& outputImage);

  struct OverlayQuadrilateral
  {
    std::array<Eigen::Vector3f, 4> corners;
    cv::Vec3f color{255.0f, 255.0f, 255.0f};
  };

  bool Render(const Eigen::Isometry3f& cameraFromWorldTransform,
              const std::vector<Eigen::Vector3f>& worldPoints,
              const std::vector<OverlayQuadrilateral>& overlays,
              cv::Mat& outputImage);

private:
  struct ProjectedPoint
  {
    int x{0};
    int y{0};
    float depth{0.0f};
  };

  struct DepthSample
  {
    float depth;
    std::size_t index;
  };

  struct ImageBuffers
  {
    std::vector<float> depthBuffer;
    std::vector<cv::Vec3f> colorBuffer;
    std::vector<float> weightBuffer;
  };

  static void ResizeBuffers(const CameraModel& cameraModel, ImageBuffers& imageBuffers);
  static bool CanRender(const CameraModel& cameraModel);
  static void PrepareRender(const CameraModel& cameraModel,
                            ImageBuffers& imageBuffers,
                            cv::Mat& outputImage);
  static void ProjectMapPoints(const CameraModel& cameraModel,
                               const Eigen::Isometry3f& cameraFromWorldTransform,
                               const std::vector<Eigen::Vector3f>& worldPoints,
                               std::vector<ProjectedPoint>& projectedPoints);
  static void RenderOverlayQuadrilateral(const CameraModel& cameraModel,
                                         const OverlayQuadrilateral& overlay,
                                         ImageBuffers& imageBuffers);
  static void ComputeNormalizedDepths(const std::vector<ProjectedPoint>& projectedPoints,
                                      std::vector<DepthSample>& depthBuffer,
                                      std::vector<float>& normalizedDepths);
  static void RenderProjectedPoint(const CameraModel& cameraModel,
                                   const ProjectedPoint& point,
                                   float normalizedDepth,
                                   float averageFocal,
                                   ImageBuffers& imageBuffers);
  static void ComposeOutputImage(const CameraModel& cameraModel,
                                 const ImageBuffers& imageBuffers,
                                 cv::Mat& outputImage);

  // Initialization parameters
  CameraModel m_cameraModel;

  // Caches
  std::vector<ProjectedPoint> m_projectedPoints;
  std::vector<DepthSample> m_depthBuffer;
  std::vector<float> m_normalizedDepths;
  ImageBuffers m_imageBuffers;
};

} // namespace SLAM
} // namespace OASIS
