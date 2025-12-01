/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "apriltag/AprilTagVisualizer.h"

#include "apriltag/AprilTagGenerator.h"

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;

namespace
{
// Drawing colours
const cv::Scalar OUTLINE_COLOR(0, 255, 255, 255); // Aqua in RGBA

constexpr int OUTLINE_THICKNESS = 16;

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

std::vector<cv::Point> CreateOutline(const std::array<apriltag_msgs::msg::Point, 4>& corners,
                                     float offsetDistance)
{
  std::array<cv::Point2f, 4> cvCorners{};
  for (std::size_t i = 0; i < cvCorners.size(); ++i)
  {
    cvCorners[i].x = static_cast<float>(corners[i].x);
    cvCorners[i].y = static_cast<float>(corners[i].y);
  }
  return CreateOutline(cvCorners, offsetDistance);
}
} // namespace

AprilTagVisualizer::AprilTagVisualizer(const rclcpp::Logger& logger,
                                       const rclcpp::Clock::SharedPtr& clock)
  : m_logger(logger), m_clock(clock), m_tagGenerator(std::make_unique<AprilTagGenerator>(logger))
{
}

AprilTagVisualizer::~AprilTagVisualizer() = default;

sensor_msgs::msg::Image::SharedPtr AprilTagVisualizer::ProcessImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
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

  return CreateOutputMessage(m_latestHeader, m_latestEncoding, m_mergedImage);
}

sensor_msgs::msg::Image::SharedPtr AprilTagVisualizer::ProcessDetections(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& msg)
{
  if (!msg || m_latestImage.empty())
    return {};

  m_overlayImage = cv::Mat::zeros(m_latestImage.size(), m_latestImage.type());

  for (const apriltag_msgs::msg::AprilTagDetection& detection : msg->detections)
  {
    const std::vector<cv::Point> outline =
        CreateOutline(detection.corners, static_cast<float>(OUTLINE_THICKNESS));

    if (!outline.empty())
    {
      cv::polylines(m_overlayImage, outline, true, OUTLINE_COLOR, OUTLINE_THICKNESS, cv::LINE_AA);

      const int32_t detectionId = detection.id;
      const cv::Mat tagImage = m_tagGenerator->Generate(detection.family, detectionId);
      if (!tagImage.empty())
      {
        OverlayTag(tagImage, ToCvCorners(detection.corners));
      }
    }
  }

  cv::addWeighted(m_latestImage, 1.0, m_overlayImage, 1.0, 0.0, m_mergedImage, -1);

  return CreateOutputMessage(m_latestHeader, m_latestEncoding, m_mergedImage);
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

sensor_msgs::msg::Image::SharedPtr AprilTagVisualizer::CreateOutputMessage(
    const std_msgs::msg::Header& latestHeader,
    const std::string& latestEncoding,
    const cv::Mat& mergedImage)
{
  cv_bridge::CvImage output(latestHeader, latestEncoding, mergedImage);
  return output.toImageMsg();
}

std::array<cv::Point2f, 4> AprilTagVisualizer::ToCvCorners(
    const std::array<apriltag_msgs::msg::Point, 4>& corners)
{
  std::array<cv::Point2f, 4> cvCorners{};
  for (std::size_t i = 0; i < corners.size(); ++i)
  {
    cvCorners[i].x = static_cast<float>(corners[i].x);
    cvCorners[i].y = static_cast<float>(corners[i].y);
  }
  return cvCorners;
}

void AprilTagVisualizer::OverlayTag(const cv::Mat& tagImage,
                                    const std::array<cv::Point2f, 4>& corners)
{
  if (tagImage.empty() || m_overlayImage.empty())
    return;

  cv::Mat tagColor;
  if (m_overlayImage.channels() == 4)
    cv::cvtColor(tagImage, tagColor, cv::COLOR_GRAY2BGRA);
  else if (m_overlayImage.channels() == 3)
    cv::cvtColor(tagImage, tagColor, cv::COLOR_GRAY2BGR);
  else if (m_overlayImage.channels() == 1)
    tagColor = tagImage;
  else
    return;

  const std::array<cv::Point2f, 4> sourceCorners = {
      cv::Point2f(0.0F, 0.0F), cv::Point2f(static_cast<float>(tagColor.cols - 1), 0.0F),
      cv::Point2f(static_cast<float>(tagColor.cols - 1), static_cast<float>(tagColor.rows - 1)),
      cv::Point2f(0.0F, static_cast<float>(tagColor.rows - 1))};

  const cv::Mat homography = cv::getPerspectiveTransform(sourceCorners.data(), corners.data());

  cv::Mat warpedTag = cv::Mat::zeros(m_overlayImage.size(), m_overlayImage.type());
  cv::Mat warpedMask = cv::Mat::zeros(m_overlayImage.size(), CV_8UC1);

  cv::Mat tagMask(tagColor.size(), CV_8UC1, cv::Scalar(255));

  cv::warpPerspective(tagColor, warpedTag, homography, m_overlayImage.size(), cv::INTER_NEAREST,
                      cv::BORDER_CONSTANT);
  cv::warpPerspective(tagMask, warpedMask, homography, m_overlayImage.size(), cv::INTER_NEAREST,
                      cv::BORDER_CONSTANT);

  warpedTag.copyTo(m_overlayImage, warpedMask);
}
