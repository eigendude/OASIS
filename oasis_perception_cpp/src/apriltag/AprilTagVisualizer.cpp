/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "apriltag/AprilTagVisualizer.h"

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

using apriltag_msgs::msg::AprilTagDetectionArray;
using sensor_msgs::msg::Image;

namespace
{
// Drawing colours
const cv::Scalar OUTLINE_COLOR(255, 255, 0, 255); // Aqua

constexpr int OUTLINE_THICKNESS = 4;

std::vector<cv::Point> CreateOutline(const std::array<cv::Point2f, 4>& corners,
                                     float offsetDistance)
{
  std::vector<cv::Point> outline;
  outline.reserve(corners.size());

  double area = 0.0;
  for (std::size_t index = 0; index < corners.size(); ++index)
  {
    const auto& current = corners[index];
    const auto& next = corners[(index + 1) % corners.size()];
    area += current.x * next.y - next.x * current.y;
  }

  const bool isCounterClockwise = area > 0.0;

  std::array<cv::Point2f, 4> outwardNormals{};
  for (std::size_t index = 0; index < corners.size(); ++index)
  {
    const auto& current = corners[index];
    const auto& next = corners[(index + 1) % corners.size()];

    const cv::Point2f edge = next - current;
    const float length = std::hypot(edge.x, edge.y);

    if (length == 0.0F)
      continue;

    if (isCounterClockwise)
      outwardNormals[index] = cv::Point2f(edge.y / length, -edge.x / length);
    else
      outwardNormals[index] = cv::Point2f(-edge.y / length, edge.x / length);
  }

  for (std::size_t index = 0; index < corners.size(); ++index)
  {
    const cv::Point2f combinedNormal =
        outwardNormals[index] + outwardNormals[(index + corners.size() - 1) % corners.size()];

    const float magnitude = std::hypot(combinedNormal.x, combinedNormal.y);

    const cv::Point2f direction =
        magnitude > 0.0F ? combinedNormal * (1.0F / magnitude) : outwardNormals[index];

    outline.emplace_back(
        static_cast<int>(std::round(corners[index].x + direction.x * offsetDistance)),
        static_cast<int>(std::round(corners[index].y + direction.y * offsetDistance)));
  }

  return outline;
}
} // namespace

using namespace OASIS;

AprilTagVisualizer::AprilTagVisualizer(const rclcpp::Logger& logger,
                                       const rclcpp::Clock::SharedPtr& clock)
  : m_logger(logger), m_clock(clock)
{
}

AprilTagVisualizer::~AprilTagVisualizer() = default;

sensor_msgs::msg::Image::SharedPtr AprilTagVisualizer::ProcessImage(
    const Image::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return nullptr;

  cv_bridge::CvImagePtr cvImage;

  try
  {
    cvImage = cv_bridge::toCvCopy(msg);
  }
  catch (const cv_bridge::Exception& exception)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", exception.what());
    return nullptr;
  }

  m_latestImage = cvImage->image;
  m_latestHeader = msg->header;
  m_latestEncoding = msg->encoding;

  if (m_overlayImage.empty())
  {
    m_mergedImage = m_latestImage;
  }
  else
  {
    cv::addWeighted(m_latestImage, 1.0, m_overlayImage, 1.0, 0.0, m_mergedImage, -1);
  }

  return CreateOutputMessage();
}

sensor_msgs::msg::Image::SharedPtr AprilTagVisualizer::ProcessDetections(
    const AprilTagDetectionArray::ConstSharedPtr& msg)
{
  if (msg == nullptr || m_latestImage.empty())
    return nullptr;

  m_overlayImage = cv::Mat::zeros(m_latestImage.size(), m_latestImage.type());

  for (const auto& detection : msg->detections)
  {
    const auto outline = CreateOutline(detection.corners, static_cast<float>(OUTLINE_THICKNESS));

    if (!outline.empty())
    {
      cv::polylines(m_overlayImage, outline, true, OUTLINE_COLOR, OUTLINE_THICKNESS, cv::LINE_AA);
    }
  }

  cv::addWeighted(m_latestImage, 1.0, m_overlayImage, 1.0, 0.0, m_mergedImage, -1);

  return CreateOutputMessage();
}

std::array<double, 2> AprilTagVisualizer::Project(const std::array<double, 9>& homography,
                                                  const std::array<double, 2>& pointInCamera)
{
  std::array<double, 2> pointInImage{};

  const auto z = homography[3 * 2 + 0] * pointInCamera[0] +
                 homography[3 * 2 + 1] * pointInCamera[1] + homography[3 * 2 + 2];

  for (std::size_t index = 0; index < 2; ++index)
  {
    pointInImage[index] =
        (homography[3 * index + 0] * pointInCamera[0] +
         homography[3 * index + 1] * pointInCamera[1] + homography[3 * index + 2]) /
        z;
  }

  return pointInImage;
}

sensor_msgs::msg::Image::SharedPtr AprilTagVisualizer::CreateOutputMessage()
{
  if (m_latestImage.empty())
    return nullptr;

  cv_bridge::CvImage output(m_latestHeader, m_latestEncoding, m_mergedImage);
  return output.toImageMsg();
}
