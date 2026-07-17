/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

#include <opencv2/core/mat.hpp>

namespace OASIS::Visualization
{
/** Configuration for perspective OLED rendering */
struct OledVisualizerConfig
{
  /** Positive output width in pixels; the source image must match */
  int width;

  /** Positive output height in pixels; the source image must match */
  int height;

  /** Positive perspective focal length in model units used for projection */
  double focal_length;

  /** Positive camera-to-plane-center distance in model units */
  double camera_distance;

  /** Positive 3D plane scale constrained to keep the plane before the camera */
  double model_scale;
};

/** Renders a monochrome image as a perspective-rotated rectangular plane */
class OledVisualizer
{
public:
  explicit OledVisualizer(const std::string& image_path, OledVisualizerConfig config);

  /** Render the plane at rotation_angle radians about its vertical axis */
  cv::Mat Render(double rotation_angle) const;

private:
  OledVisualizerConfig config_;
  cv::Mat source_;
  cv::Mat mirrored_source_;
};
} // namespace OASIS::Visualization
