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

  void SetCameraModel(const CameraModel& model);
  void SetImageSize(int width, int height);
  void EnableRoiMask(bool enable);
  void SetRoiPolygon(const std::vector<cv::Point>& polygon);

  bool Render(const Sophus::SE3f& Tcw,
              const std::vector<ORB_SLAM3::MapPoint*>& mapPoints,
              cv::Mat& outputImage);

private:
  void ResizeBuffers();
  void UpdateRoiMask();

  CameraModel m_cameraModel;
  int m_width{0};
  int m_height{0};
  ViridisPaletteSampler m_paletteSampler;
  std::vector<float> m_depthBuffer;
  std::vector<cv::Vec3f> m_colorBuffer;
  std::vector<float> m_weightBuffer;
  bool m_roiMaskEnabled{false};
  bool m_roiMaskDirty{true};
  std::vector<cv::Point> m_roiPolygon;
  cv::Mat m_roiMask;
};

} // namespace SLAM
} // namespace OASIS
