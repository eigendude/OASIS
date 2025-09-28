/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

//#include "utils/frame_pool.hpp"

#include <stdint.h>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace OASIS
{
namespace VIDEO
{
class VisionGraph;

struct ConfigOptions
{
  // The maximum number of points to track
  unsigned int maxPointCount = 200;

  // The maximum number of frames to solve for
  unsigned int maxFrameCount = 240;
};

struct FrameInfo
{
  double sceneScore = 0.0; // in the range [0.0, 1.0], 1.0 is a new scene
  uintptr_t pointData = 0;
  unsigned int pointSize = 0;
  uintptr_t initialPointData = 0;
  unsigned int initialPointSize = 0;
};

class OpticalFlow
{
public:
  OpticalFlow() = default;
  ~OpticalFlow() { Deinitialize(); }

  /*!
   * \brief Initialize the motion tracker with the specified dimensions
   *
   * \param width The video width
   * \param height The video height
   */
  bool Initialize(int width, int height);

  /*!
   * \brief Deinitialize the motion tracker
   */
  void Deinitialize() {}

  void SetConfig(const ConfigOptions& config);

  bool ProcessImage(const cv::Mat& image);

  /*!
   * \brief Add a frame to the motion tracker and return the results
   *
   * \param frameArray Javascript array of type Uint8ClampedArray
   *
   * \return Results of analyzing the new frame
   *
  FrameInfo AddVideoFrame(const emscripten::val& frameArray);
  */

  std::vector<float> GetPoints() const { return m_points; }
  std::vector<float> GetInitialPoints() const { return m_initialPoints; }

private:
  /*!
   * \brief Convert a 32-bit RGBA frame to 8-bit grayscale
   */
  void ConvertToGrayscale(const cv::Mat& in, cv::Mat& out);

  /*!
   * \brief Find features in a frame that will be good for tracking
   */
  void FindFeatures(const cv::Mat& currentGrayscale,
                    std::vector<cv::Point2f>& currentPoints,
                    std::vector<uint8_t>& status,
                    std::vector<float>& errors);

  /*!
   * \brief Calculates the optical flow between the last two frames
   */
  void CalculateOpticalFlow(const cv::Mat& currentGrayscale,
                            std::vector<cv::Point2f>& currentPoints,
                            std::vector<uint8_t>& status,
                            std::vector<float>& errors);

  /*!
   * \brief Add frame to history vector
   *
  void AddFrameToHistory(FramePtr&& frame);

  /*!
   * \brief Fill out the frame struct being returned to JavaScript land
   */
  FrameInfo GetResult() const;

  // Video parameters
  unsigned int m_width = 0;
  unsigned int m_height = 0;

  // Config parameters
  ConfigOptions m_config;

  // State parameters
  //std::vector<FramePtr> m_frameHistory;
  cv::Mat m_previousGrayscale;

  // Vision graph
  std::shared_ptr<VisionGraph> m_visionGraph;

  // Frame pool
  //std::shared_ptr<FramePool> m_framePool;

  // Buffers
  cv::Mat m_rgbaFrameBuffer;
  cv::Mat m_currentGrayscaleBuffer;
  std::vector<std::vector<cv::Point2f>> m_pointHistoryBuffer;
  std::vector<uint8_t> m_statusBuffer;

  bool m_hasPreviousFrame{false};
  std::vector<cv::Point2f> m_previousPoints;

  // Output buffer (holds data returned from AddVideoFrame())
  mutable std::vector<float> m_points;
  mutable std::vector<float> m_initialPoints;
};
} // namespace VIDEO
} // namespace OASIS
