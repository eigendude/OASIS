/*
 *  Copyright (C) 2020-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "OpticalFlow.h"

#include "utils/MathUtils.h"
#include "utils/SceneUtils.h"
#include "video/VisionGraph.h"

#include <algorithm>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

using namespace OASIS;
using namespace VIDEO;

namespace
{
// Minimum number of points to force redetection
constexpr unsigned int MIN_POINT_COUNT = 50;
} // namespace

bool OpticalFlow::Initialize(rclcpp::Logger& logger,
                             int width,
                             int height,
                             bool calculateSceneScores)
{
  if (width <= 0 || height <= 0)
  {
    RCLCPP_ERROR(logger, "Invalid width/height: %d x %d", width, height);
    return false;
  }

  // Initialize video parameters
  m_width = width;
  m_height = height;
  m_calculateSceneScores = calculateSceneScores;

  // Initialize buffers
  m_rgbaFrameBuffer.create(m_height, m_width, CV_8UC4);
  m_currentGrayscaleBuffer.create(m_height, m_width, CV_8UC1);
  m_previousGrayscale.create(m_height, m_width, CV_8UC1);

  m_visionGraph.reset(new VisionGraph);
  m_visionGraph->Compile(width, height, m_rgbaFrameBuffer, m_currentGrayscaleBuffer,
                         m_previousGrayscale);

  m_hasPreviousFrame = false;
  m_previousPoints.clear();
  m_points.clear();
  m_initialPoints.clear();
  m_previousMafd = 0.0f;
  m_sceneScore = 0.0f;
  m_hasSceneScore = false;

  RCLCPP_INFO(logger, "Initialized optical flow with dimensions %u x %u", width, height);

  return true;
}

void OpticalFlow::Deinitialize()
{
  m_visionGraph.reset();

  m_rgbaFrameBuffer.release();
  m_currentGrayscaleBuffer.release();
  m_previousGrayscale.release();
}

void OpticalFlow::SetConfig(const ConfigOptions& config)
{
  m_config = config;
}

bool OpticalFlow::ProcessImage(const cv::Mat& image)
{
  if (!m_visionGraph)
    return false;

  if (image.empty())
    return false;

  if (image.cols != static_cast<int>(m_width) || image.rows != static_cast<int>(m_height))
    return false;

  cv::Mat& rgbaFrame = m_rgbaFrameBuffer;
  switch (image.type())
  {
    case CV_8UC4:
      image.copyTo(rgbaFrame);
      break;
    case CV_8UC3:
      cv::cvtColor(image, rgbaFrame, cv::COLOR_BGR2RGBA);
      break;
    case CV_8UC1:
      cv::cvtColor(image, rgbaFrame, cv::COLOR_GRAY2RGBA);
      break;
    default:
      return false;
  }

  cv::Mat& currentGrayscale = m_currentGrayscaleBuffer;
  ConvertToGrayscale(rgbaFrame, currentGrayscale);

  std::vector<cv::Point2f> currentPoints;
  std::vector<uint8_t> status;
  std::vector<float> errors;

  float nextMafd = 0.0f;
  if (m_calculateSceneScores)
  {
    if (m_hasPreviousFrame)
    {
      nextMafd = UTILS::SceneUtils::CalcSceneMAFD(m_previousGrayscale.data, currentGrayscale.data,
                                                  m_width, m_height);
      m_sceneScore = UTILS::SceneUtils::CalcSceneScore(nextMafd, m_previousMafd);
      m_hasSceneScore = true;
    }
    else
    {
      m_sceneScore = 0.0f;
      m_hasSceneScore = false;
    }
  }
  else
  {
    m_sceneScore = 0.0f;
    m_hasSceneScore = false;
  }

  auto storeInitialPoints = [this](const std::vector<cv::Point2f>& points)
  {
    m_initialPoints.clear();
    if (points.empty())
      return;

    m_initialPoints.reserve(points.size() * 2);
    for (const cv::Point2f& point : points)
    {
      m_initialPoints.push_back(point.x);
      m_initialPoints.push_back(point.y);
    }
  };

  if (!m_hasPreviousFrame || m_previousPoints.size() <= MIN_POINT_COUNT)
  {
    FindFeatures(currentGrayscale, currentPoints, status, errors);
    storeInitialPoints(currentPoints);
  }
  else
  {
    CalculateOpticalFlow(currentGrayscale, currentPoints, status, errors);

    std::vector<cv::Point2f> filteredPoints;
    filteredPoints.reserve(currentPoints.size());
    for (size_t i = 0; i < currentPoints.size(); ++i)
    {
      if (i < status.size() && status[i])
        filteredPoints.emplace_back(currentPoints[i]);
    }
    currentPoints = std::move(filteredPoints);

    if (currentPoints.size() <= MIN_POINT_COUNT)
    {
      FindFeatures(currentGrayscale, currentPoints, status, errors);
      storeInitialPoints(currentPoints);
    }
  }

  m_points.clear();
  if (!currentPoints.empty())
  {
    m_points.reserve(currentPoints.size() * 2);
    for (const cv::Point2f& point : currentPoints)
    {
      m_points.push_back(point.x);
      m_points.push_back(point.y);
    }
  }

  currentGrayscale.copyTo(m_previousGrayscale);
  m_previousPoints = std::move(currentPoints);
  m_hasPreviousFrame = true;
  m_previousMafd = nextMafd;

  return true;
}

size_t OpticalFlow::DrawPoints(cv::Mat& image, size_t maxPointCount) const
{
  if (image.empty())
    return 0;

  const size_t trackedPointCount = std::min(m_points.size() / 2, maxPointCount);
  for (size_t index = 0; index < trackedPointCount; ++index)
  {
    const cv::Point2f point(m_points[index * 2], m_points[index * 2 + 1]);
    cv::circle(image, point, 14, cv::Scalar(255, 255, 0), 6, cv::LINE_AA);
  }

  return trackedPointCount;
}

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
  const double minDistance = std::max(UTILS::MathUtils::GeometricMean(m_width, m_height) /
                                          (static_cast<double>(m_config.maxPointCount) / 2.0),
                                      2.0);

  m_visionGraph->FindFeatures(currentGrayscale, m_config.maxPointCount, minDistance, currentPoints);
  status.assign(currentPoints.size(), static_cast<uint8_t>(1));
  errors.assign(currentPoints.size(), 0.0f);
}

void OpticalFlow::CalculateOpticalFlow(const cv::Mat& currentGrayscale,
                                       std::vector<cv::Point2f>& currentPoints,
                                       std::vector<uint8_t>& status,
                                       std::vector<float>& errors)
{
  if (!m_visionGraph)
    return;

  if (!m_hasPreviousFrame)
    return;

  if (m_previousPoints.empty())
  {
    currentPoints.clear();
    status.clear();
    errors.clear();
    return;
  }

  // Rebuild the point history with the latest set of tracked points
  m_pointHistoryBuffer.clear();
  m_pointHistoryBuffer.emplace_back(m_previousPoints);

  m_visionGraph->CalcOpticalFlow(m_previousGrayscale, currentGrayscale, m_previousPoints,
                                 m_pointHistoryBuffer, currentPoints, status, errors);
}
