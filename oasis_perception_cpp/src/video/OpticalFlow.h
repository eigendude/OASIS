/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

//#include "utils/frame_pool.hpp"

#include <limits>
#include <stdint.h>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace rclcpp
{
class Logger;
} // namespace rclcpp

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

class OpticalFlow
{
public:
  OpticalFlow() = default;
  ~OpticalFlow() { Deinitialize(); }

  /*!
   * \brief Initialize the motion tracker with the specified dimensions
   *
   * \param logger The ROS logger
   * \param width The video width
   * \param height The video height
   */
  bool Initialize(rclcpp::Logger& logger, int width, int height);

  /*!
   * \brief Deinitialize the motion tracker
   */
  void Deinitialize() {}

  void SetConfig(const ConfigOptions& config);

  bool ProcessImage(const cv::Mat& image);

  //! \brief Draw tracked points onto an image
  size_t DrawPoints(cv::Mat& image,
                    size_t maxPointCount = std::numeric_limits<size_t>::max()) const;

  std::vector<float> GetPoints() const { return m_points; }
  std::vector<float> GetInitialPoints() const { return m_initialPoints; }

  float GetSceneScore() const { return m_sceneScore; }
  float GetSceneMafd() const { return m_previousMafd; }
  bool HasSceneScore() const { return m_hasSceneScore; }

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

  // Video parameters
  unsigned int m_width = 0;
  unsigned int m_height = 0;

  // Config parameters
  ConfigOptions m_config;

  // State parameters
  cv::Mat m_previousGrayscale;

  // Vision graph
  std::shared_ptr<VisionGraph> m_visionGraph;

  // Buffers
  cv::Mat m_rgbaFrameBuffer;
  cv::Mat m_currentGrayscaleBuffer;
  std::vector<std::vector<cv::Point2f>> m_pointHistoryBuffer;

  bool m_hasPreviousFrame{false};
  std::vector<cv::Point2f> m_previousPoints;

  // Output buffer (holds data returned from AddVideoFrame())
  mutable std::vector<float> m_points;
  mutable std::vector<float> m_initialPoints;

  // Scene detection state
  float m_previousMafd{0.0f};
  float m_sceneScore{0.0f};
  bool m_hasSceneScore{false};
};
} // namespace VIDEO
} // namespace OASIS
