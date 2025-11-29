/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "apriltag/AprilTagVisualizer.h"

#include <array>
#include <string>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

using apriltag_msgs::msg::AprilTagDetectionArray;
using sensor_msgs::msg::Image;

namespace
{
// Drawing colours
const std::array<cv::Scalar, 4> COLORS = {
    cv::Scalar(0, 0, 255, 255), // Red
    cv::Scalar(0, 255, 0, 255), // Green
    cv::Scalar(255, 0, 0, 255), // Blue
    cv::Scalar(0, 255, 255, 255), // Yellow
};
} // namespace

using namespace OASIS;

AprilTagVisualizer::AprilTagVisualizer(const rclcpp::Logger& logger,
                                       const rclcpp::Clock::SharedPtr& clock)
  : m_logger(logger),
    m_clock(clock)
{
}

AprilTagVisualizer::~AprilTagVisualizer() = default;

void AprilTagVisualizer::SetOverlayMode(const std::string& overlayMode)
{
  m_overlayMode = overlayMode;
}

void AprilTagVisualizer::SetAlpha(double alpha)
{
  m_alpha = alpha;
}

sensor_msgs::msg::Image::SharedPtr
AprilTagVisualizer::ProcessImage(const Image::ConstSharedPtr& msg)
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
    cv::addWeighted(m_latestImage, 1.0, m_overlayImage, m_alpha, 0.0, m_mergedImage, -1);
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
    if (m_overlayMode == "axes")
    {
      const auto center = Project(detection.homography, {0.0, 0.0});
      const auto xAxis = Project(detection.homography, {1.0, 0.0});
      const auto yAxis = Project(detection.homography, {0.0, 1.0});

      cv::line(m_overlayImage, cv::Point2d(center[0], center[1]), cv::Point2d(xAxis[0], xAxis[1]),
               COLORS[0], 3);
      cv::line(m_overlayImage, cv::Point2d(center[0], center[1]), cv::Point2d(yAxis[0], yAxis[1]),
               COLORS[1], 3);
    }
    else if (m_overlayMode == "tri")
    {
      std::array<cv::Point, 3> points;
      points[0].x = detection.centre.x;
      points[0].y = detection.centre.y;

      for (std::size_t index = 0; index < 4; ++index)
      {
        points[1].x = detection.corners[index % 4].x;
        points[1].y = detection.corners[index % 4].y;
        points[2].x = detection.corners[(index + 1) % 4].x;
        points[2].y = detection.corners[(index + 1) % 4].y;

        cv::fillConvexPoly(m_overlayImage, points.data(), 3, COLORS[index]);
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 5000, "Unknown overlay mode: %s", m_overlayMode.c_str());
      break;
    }

    for (std::size_t index = 0; index < 4; ++index)
    {
      cv::circle(m_overlayImage, cv::Point(detection.corners[index].x, detection.corners[index].y), 5,
                 COLORS[index], 2);
    }
  }

  cv::addWeighted(m_latestImage, 1.0, m_overlayImage, m_alpha, 0.0, m_mergedImage, -1);

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
