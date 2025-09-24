/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "OpticalFlow.h"

#include "video/VisionGraph.h"
//#include "utils/math_utils.hpp"

#include <algorithm>
#include <iostream>

#include <opencv2/video/tracking.hpp>

using namespace oasis_perception;

namespace
{
// Minimum number of points to force redetection
constexpr unsigned int MIN_POINT_COUNT = 5;
} // namespace

bool OpticalFlow::Initialize(int width, int height)
{
  if (width <= 0 || height <= 0)
    return false;

  // Initialize video parameters
  m_width = width;
  m_height = height;

  // Initialize buffers
  m_rgbaFrameBuffer.create(m_height, m_width, CV_8UC4);
  m_currentGrayscaleBuffer.create(m_height, m_width, CV_8UC1);
  m_previousGrayscale.create(m_height, m_width, CV_8UC1);

  // Focal length of the camera
  const double f = 1082.77717353143; // TODO

  // Principle point of the camera
  cv::Point2d pp(static_cast<double>(m_width),
                 static_cast<double>(m_height)); // TODO

  // Initialize camera calibration matrix with sensible values
  m_cameraMatrix = (cv::Mat_<double>(3, 3) << f, 0, pp.x, 0, f, pp.y, 0, 0, 1.0);

  m_visionGraph.reset(new VisionGraph);
  m_visionGraph->Compile(width, height, m_rgbaFrameBuffer, m_currentGrayscaleBuffer,
                         m_previousGrayscale);

  //m_framePool.reset(new FramePool);

  return true;
}

void OpticalFlow::SetConfig(const ConfigOptions& config)
{
  m_config = config;
}

/*
FrameInfo OpticalFlow::AddVideoFrame(const emscripten::val& frameArray)
{
  // Dereference buffers
  cv::Mat& rgbaFrame = m_rgbaFrameBuffer;
  cv::Mat& currentGrayscale = m_currentGrayscaleBuffer;

  // Get a frame to gather our results
  FramePtr currentFrame = m_framePool->GetFrame();

  // Fetch array from JavaScript
  // TODO: Elide copy
  ReadArray(frameArray, rgbaFrame.data);

  // Convert to grayscale
  ConvertToGrayscale(rgbaFrame, currentGrayscale);

  // Reset frame history when a scene change is detected
  if (currentFrame->sceneScore >= SCENE_THREASHOLD)
  {
    std::cout << "Scene change detected (score: " << currentFrame->sceneScore << ")" << std::endl;
    m_frameHistory.clear();
  }

  // TODO
  if (m_frameHistory.size() > m_config.maxFrameCount)
  {
    m_frameHistory.clear();
  }

  if (m_frameHistory.empty())
  {
    FindFeatures(currentGrayscale, currentFrame->points, currentFrame->status,
                 currentFrame->errors);
  }
  else
  {
    // Calculate optical flow if we have a previous frame
    CalculateOpticalFlow(currentGrayscale, currentFrame->points, currentFrame->status,
                         currentFrame->errors);

    // If feature count drops by 10% or more, consider it a scene change
    const unsigned int missing = std::count(currentFrame->status.begin(),
                                            currentFrame->status.end(),
                                            0);

    if (10 * missing > currentFrame->status.size())
    {
      std::cout << "Scene change detected (missing points: " << missing << ")"
          << std::endl;
      m_frameHistory.clear();
    }

    if (currentFrame->points.size() <= MIN_POINT_COUNT)
    {
      std::cout << "Scene change detected (points count: " << currentFrame->points.size()
          << ")" << std::endl;
      m_frameHistory.clear();
    }

    if (m_frameHistory.empty())
      FindFeatures(currentGrayscale, currentFrame->points, currentFrame->status, currentFrame->errors);
  }

  if (!currentFrame->points.empty())
    AddFrameToHistory(std::move(currentFrame));

  // Update state
  std::swap(currentGrayscale, m_previousGrayscale);

  // Create result
  FrameInfo frameInfo = GetResult();

  return frameInfo;
}
*/

void OpticalFlow::ConvertToGrayscale(const cv::Mat& in, cv::Mat& out)
{
  m_visionGraph->ApplyGrayscale(in, out);
}

void OpticalFlow::FindFeatures(const cv::Mat& currentGrayscale,
                               std::vector<cv::Point2f>& currentPoints,
                               std::vector<uint8_t>& status,
                               std::vector<float>& errors)
{
  // TODO
  const double minDistance = std::max(MathUtils::GeometricMean(m_width, m_height) /
                                          (static_cast<double>(m_config.maxPointCount) / 2.0),
                                      2.0);

  m_visionGraph->FindFeatures(currentGrayscale, m_config.maxPointCount, minDistance, currentPoints);
  status.assign(currentPoints.size(), 1U);
  errors.assign(currentPoints.size(), 0.0f);
}

void OpticalFlow::CalculateOpticalFlow(const cv::Mat& currentGrayscale,
                                       std::vector<cv::Point2f>& currentPoints,
                                       std::vector<uint8_t>& status,
                                       std::vector<float>& errors)
{
  /*
  if (!m_frameHistory.empty())
  {
    const std::vector<cv::Point2f>& previousPoints = m_frameHistory.back()->points;
    if (!previousPoints.empty())
    {
      // TODO: Zero-copy
      m_pointHistoryBuffer.clear();
      for (const auto& frame : m_frameHistory)
        m_pointHistoryBuffer.emplace_back(frame->points);

      m_visionGraph->CalcOpticalFlow(m_previousGrayscale, currentGrayscale, previousPoints,
                                     m_pointHistoryBuffer, currentPoints, status, errors);
    }
  }
  */
}

/*
void OpticalFlow::AddFrameToHistory(FramePtr&& frame)
{
  // Check for missing points (value of "status" is 0)
  std::vector<unsigned int> missing;
  for (unsigned int index = 0; index < frame->status.size(); index++)
  {
    if (frame->status[index] == 0)
      missing.push_back(index);
  }

  m_frameHistory.emplace_back(std::move(frame));

  if (!missing.empty())
  {
    // Prune missing points from history
    for (auto& frame : m_frameHistory)
    {
      if (frame->points.empty())
        continue;

      // This used to use lambdas, but they were causing function index
      // out-of-bound errors in the browser
      for (auto it = missing.end(); it != missing.begin(); --it)
      {
        const unsigned int index = *(it - 1);

        frame->points.erase(frame->points.begin() + index);
        frame->status.erase(frame->status.begin() + index);
        frame->errors.erase(frame->errors.begin() + index);
      }
    }
  }
}
*/

FrameInfo OpticalFlow::GetResult() const
{
  FrameInfo frameInfo{};

  m_points.clear();
  m_initialPoints.clear();

  /*
  if (!m_frameHistory.empty())
  {
    // Grab references to pertinent frames
    const FramePtr& currentFrame = m_frameHistory.back();
    const FramePtr& initialFrame = m_frameHistory.front();

    // Set current points
    const std::vector<cv::Point2f>& currentPoints = currentFrame->points;
    if (!currentPoints.empty())
    {
      m_points.reserve(currentPoints.size() * 2);
      for (const cv::Point2f& point : currentPoints)
      {
        m_points.push_back(point.x);
        m_points.push_back(point.y);
      }
      frameInfo.pointData = reinterpret_cast<uintptr_t>(m_points.data());
      frameInfo.pointSize = m_points.size();
    }

    // Set initial points
    const std::vector<cv::Point2f>& initialPoints = initialFrame->points;
    if (!initialPoints.empty())
    {
      m_initialPoints.reserve(initialPoints.size() * 2);
      for (const cv::Point2f& point : initialPoints)
      {
        m_initialPoints.push_back(point.x);
        m_initialPoints.push_back(point.y);
      }
      frameInfo.initialPointData = reinterpret_cast<uintptr_t>(m_initialPoints.data());
      frameInfo.initialPointSize = m_initialPoints.size();
    }
  }
  */

  return frameInfo;
}
