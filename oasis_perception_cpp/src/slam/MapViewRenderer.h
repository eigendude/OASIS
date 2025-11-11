/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "CameraModel.h"
#include "ViridisPaletteSampler.h"

#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

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

  void Initialize(const CameraModel& model);
  void SetImageSize(int width, int height);

  bool Render(const Sophus::SE3f& cameraFromWorldTransform,
              const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
              cv::Mat& outputImage);

private:
  struct ProjectedPoint
  {
    int x{0};
    int y{0};
    float depth{0.0f};
  };

  bool CanRender() const;
  void PrepareRender(cv::Mat& outputImage);
  std::vector<ProjectedPoint> ProjectMapPoints(
      const Sophus::SE3f& cameraFromWorldTransform,
      const std::vector<ORB_SLAM3::MapPoint*>& mapPoints) const;
  std::vector<float> ComputeNormalizedDepths(const std::vector<ProjectedPoint>& projectedPoints) const;
  void RenderProjectedPoint(const ProjectedPoint& point,
                            float normalizedDepth,
                            float averageFocal);
  void ComposeOutputImage(cv::Mat& outputImage) const;
  void ResizeBuffers();

  // Initialization parameters
  CameraModel m_cameraModel;

  // Image parameters
  int m_width{0};
  int m_height{0};

  // Buffers
  std::vector<float> m_depthBuffer;
  std::vector<cv::Vec3f> m_colorBuffer;
  std::vector<float> m_weightBuffer;

  // Utilities
  ViridisPaletteSampler m_paletteSampler;
};

} // namespace SLAM
} // namespace OASIS
