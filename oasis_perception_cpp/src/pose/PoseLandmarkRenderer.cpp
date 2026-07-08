/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "PoseLandmarkRenderer.h"

#include <algorithm>
#include <cmath>

#include <mediapipe/framework/formats/landmark.pb.h>
#include <mediapipe/util/pose_util.h>
#include <opencv2/imgproc.hpp>

using namespace oasis_perception;

namespace
{
const cv::Scalar BOUNDING_BOX_COLOR{255.0, 255.0, 0.0};

// Pixel thickness used for scene bounding boxes
constexpr int BOUNDING_BOX_THICKNESS = 3;

int ClampPixel(const float normalizedValue, const int maxPixel)
{
  if (maxPixel <= 0)
    return 0;

  const float clamped = std::clamp(normalizedValue, 0.0F, 1.0F);

  return std::clamp(static_cast<int>(std::lround(clamped * static_cast<float>(maxPixel))), 0,
                    maxPixel);
}

cv::Rect BoundingBoxToRect(const PoseBoundingBox& box, const cv::Size& size)
{
  const float xMin = box.xCenter - box.width * 0.5F;
  const float yMin = box.yCenter - box.height * 0.5F;
  const float xMax = box.xCenter + box.width * 0.5F;
  const float yMax = box.yCenter + box.height * 0.5F;

  const int left = ClampPixel(xMin, size.width - 1);
  const int top = ClampPixel(yMin, size.height - 1);
  const int right = ClampPixel(xMax, size.width - 1);
  const int bottom = ClampPixel(yMax, size.height - 1);

  return {
      std::min(left, right),
      std::min(top, bottom),
      std::max(1, std::abs(right - left)),
      std::max(1, std::abs(bottom - top)),
  };
}

mediapipe::NormalizedLandmarkList ToMediaPipeLandmarks(
    const std::span<const PoseLandmark> landmarks)
{
  mediapipe::NormalizedLandmarkList landmarkList;

  for (const PoseLandmark& landmarkInput : landmarks)
  {
    mediapipe::NormalizedLandmark* landmark = landmarkList.add_landmark();
    landmark->set_x(landmarkInput.x);
    landmark->set_y(landmarkInput.y);
    landmark->set_z(landmarkInput.z);
  }

  return landmarkList;
}
} // namespace

void PoseLandmarkRenderer::Render(cv::Mat& image,
                                  const std::span<const PoseRenderInput> poses) const
{
  if (image.empty())
    return;

  const cv::Size imageSize = image.size();

  for (const PoseRenderInput& pose : poses)
  {
    mediapipe::NormalizedLandmarkList landmarkList = ToMediaPipeLandmarks(pose.landmarks);
    mediapipe::DrawPose(landmarkList, false, &image);

    if (pose.hasBoundingBox)
    {
      cv::rectangle(image, BoundingBoxToRect(pose.boundingBox, imageSize), BOUNDING_BOX_COLOR,
                    BOUNDING_BOX_THICKNESS);
    }
  }
}
