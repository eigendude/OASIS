/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <stdint.h>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/gapi/gcompiled.hpp>

namespace oasis_perception
{
class VisionGraph
{
public:
  VisionGraph() = default;
  ~VisionGraph();

  void Compile(unsigned int width,
               unsigned int height,
               const cv::Mat& currentFrame,
               const cv::Mat& currentGrayscale,
               const cv::Mat& previousGrayscale);

  void ApplyGrayscale(
      // Input
      const cv::Mat& currentFrame,
      // Output
      cv::Mat& currentGrayscale);

  void FindFeatures(
      // Input
      const cv::Mat& currentGrayscale,
      unsigned int maxFeatures,
      double minDistance,
      // Output
      std::vector<cv::Point2f>& currentPoints);

  void CalcOpticalFlow(
      // Input
      const cv::Mat& previousGrayscale,
      const cv::Mat& currentGrayscale,
      const std::vector<cv::Point2f>& previousPoints,
      const std::vector<std::vector<cv::Point2f>>& pointHistory,
      // Output
      std::vector<cv::Point2f>& currentPoints,
      std::vector<uchar>& status,
      std::vector<float>& errors);

private:
  cv::GCompiled m_applyGrayscale;
  cv::GCompiled m_findFeatures;
  cv::GCompiled m_calcOpticalFlow;
};
} // namespace oasis_perception
