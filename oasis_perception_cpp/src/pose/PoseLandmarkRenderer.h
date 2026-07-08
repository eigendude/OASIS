/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <span>
#include <vector>

#include <opencv2/core.hpp>

namespace oasis_perception
{
/**
 * A single normalized pose landmark
 */
struct PoseLandmark
{
  // Normalized image x coordinate, [0, 1], left to right
  float x = 0.0F;

  // Normalized image y coordinate, [0, 1], top to bottom
  float y = 0.0F;

  // MediaPipe model-relative depth; lower values are closer to the camera
  float z = 0.0F;
};

/**
 * A normalized 2D bounding box for one pose
 */
struct PoseBoundingBox
{
  // Normalized center x coordinate, [0, 1], left to right
  float xCenter = 0.0F;

  // Normalized center y coordinate, [0, 1], top to bottom
  float yCenter = 0.0F;

  // Normalized box width, [0, 1], relative to image width
  float width = 0.0F;

  // Normalized box height, [0, 1], relative to image height
  float height = 0.0F;
};

/**
 * A renderer input for one detected pose
 */
struct PoseRenderInput
{
  // Landmarks in MediaPipe pose landmark index order
  std::vector<PoseLandmark> landmarks;

  // Optional smoothed box from the upstream scene publisher
  PoseBoundingBox boundingBox;

  // True when boundingBox contains a box for this pose
  bool hasBoundingBox = false;
};

class PoseLandmarkRenderer
{
public:
  void Render(cv::Mat& image, std::span<const PoseRenderInput> poses) const;
};
} // namespace oasis_perception
