/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/AprilTagVizNode.h"

#include <array>
#include <functional>
#include <string>

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

using apriltag_msgs::msg::AprilTagDetectionArray;
using sensor_msgs::msg::Image;

namespace
{
// Topic suffixes
constexpr std::string_view IMAGE_TOPIC = "image";
constexpr std::string_view DETECTIONS_TOPIC = "apriltags";
constexpr std::string_view OVERLAY_TOPIC = "apriltags_image";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view OVERLAY_MODE_PARAMETER = "overlay_mode";
constexpr std::string_view ALPHA_PARAMETER = "alpha";

constexpr std::string_view DEFAULT_SYSTEM_ID = "";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";
constexpr std::string_view DEFAULT_OVERLAY_MODE = "axes";
constexpr double DEFAULT_ALPHA = 0.5;

// Drawing colours
constexpr std::array<cv::Scalar, 4> COLOURS = {
    cv::Scalar(0, 0, 255, 255),    // Red
    cv::Scalar(0, 255, 0, 255),    // Green
    cv::Scalar(255, 0, 0, 255),    // Blue
    cv::Scalar(0, 255, 255, 255),  // Yellow
};
} // namespace

using namespace OASIS;

AprilTagVizNode::AprilTagVizNode(rclcpp::Node& node) : m_node(node), m_logger(node.get_logger())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<std::string>(OVERLAY_MODE_PARAMETER.data(), DEFAULT_OVERLAY_MODE.data());
  m_node.declare_parameter<double>(ALPHA_PARAMETER.data(), DEFAULT_ALPHA);
}

AprilTagVizNode::~AprilTagVizNode() = default;

bool AprilTagVizNode::Initialize()
{
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), m_systemId) || m_systemId.empty())
  {
    RCLCPP_ERROR(m_logger, "Missing or empty system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), m_imageTransport);
  m_node.get_parameter(OVERLAY_MODE_PARAMETER.data(), m_overlayMode);
  m_node.get_parameter(ALPHA_PARAMETER.data(), m_alpha);

  std::string imageTopic = m_systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string detectionsTopic = m_systemId;
  detectionsTopic.push_back('_');
  detectionsTopic.append(DETECTIONS_TOPIC);

  std::string overlayTopic = m_systemId;
  overlayTopic.push_back('_');
  overlayTopic.append(OVERLAY_TOPIC);

  RCLCPP_INFO(m_logger, "System ID: %s", m_systemId.c_str());
  RCLCPP_INFO(m_logger, "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Detections topic: %s", detectionsTopic.c_str());
  RCLCPP_INFO(m_logger, "Overlay topic: %s", overlayTopic.c_str());
  RCLCPP_INFO(m_logger, "Overlay mode: %s", m_overlayMode.c_str());
  RCLCPP_INFO(m_logger, "Alpha: %.2f", m_alpha);
  RCLCPP_INFO(m_logger, "Image transport: %s", m_imageTransport.c_str());

  rclcpp::SensorDataQoS qos;
  qos.keep_last(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  image_transport::TransportHints hints(&m_node, m_imageTransport, "", qos.get_rmw_qos_profile());

  m_overlayPublisher = std::make_unique<image_transport::Publisher>();
  *m_overlayPublisher =
      image_transport::create_publisher(&m_node, overlayTopic, qos.get_rmw_qos_profile());

  m_imageSubscription = std::make_unique<image_transport::Subscriber>();
  *m_imageSubscription = image_transport::create_subscription(
      &m_node, imageTopic, std::bind(&AprilTagVizNode::OnImage, this, std::placeholders::_1),
      hints);

  m_detectionSubscription = m_node.create_subscription<AprilTagDetectionArray>(
      detectionsTopic, qos,
      std::bind(&AprilTagVizNode::OnDetections, this, std::placeholders::_1));

  return true;
}

void AprilTagVizNode::Deinitialize()
{
  m_detectionSubscription.reset();

  if (m_imageSubscription)
  {
    m_imageSubscription->shutdown();
    m_imageSubscription.reset();
  }

  if (m_overlayPublisher)
  {
    m_overlayPublisher->shutdown();
    m_overlayPublisher.reset();
  }
}

void AprilTagVizNode::OnImage(const Image::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  cv_bridge::CvImagePtr cvImage;

  try
  {
    cvImage = cv_bridge::toCvCopy(msg);
  }
  catch (const cv_bridge::Exception& exception)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", exception.what());
    return;
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

  if (m_overlayPublisher)
  {
    cv_bridge::CvImage output(m_latestHeader, m_latestEncoding, m_mergedImage);
    m_overlayPublisher->publish(output.toImageMsg());
  }
}

void AprilTagVizNode::OnDetections(const AprilTagDetectionArray::SharedPtr& msg)
{
  if (msg == nullptr || m_latestImage.empty())
    return;

  m_overlayImage = cv::Mat::zeros(m_latestImage.size(), m_latestImage.type());

  for (const auto& detection : msg->detections)
  {
    if (m_overlayMode == "axes")
    {
      const auto center = Project(detection.homography, {0.0, 0.0});
      const auto xAxis = Project(detection.homography, {1.0, 0.0});
      const auto yAxis = Project(detection.homography, {0.0, 1.0});

      cv::line(m_overlayImage, cv::Point2d(center[0], center[1]),
               cv::Point2d(xAxis[0], xAxis[1]), COLOURS[0], 3);
      cv::line(m_overlayImage, cv::Point2d(center[0], center[1]),
               cv::Point2d(yAxis[0], yAxis[1]), COLOURS[1], 3);
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

        cv::fillConvexPoly(m_overlayImage, points.data(), 3, COLOURS[index]);
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(m_logger, *m_node.get_clock(), 5000,
                           "Unknown overlay mode: %s", m_overlayMode.c_str());
      break;
    }

    for (std::size_t index = 0; index < 4; ++index)
    {
      cv::circle(m_overlayImage, cv::Point(detection.corners[index].x, detection.corners[index].y),
                 5, COLOURS[index], 2);
    }
  }

  // Refresh the merged image using the latest overlay
  cv::addWeighted(m_latestImage, 1.0, m_overlayImage, m_alpha, 0.0, m_mergedImage, -1);

  if (m_overlayPublisher)
  {
    cv_bridge::CvImage output(m_latestHeader, m_latestEncoding, m_mergedImage);
    m_overlayPublisher->publish(output.toImageMsg());
  }
}

std::array<double, 2> AprilTagVizNode::Project(const std::array<double, 9>& homography,
                                               const std::array<double, 2>& pointInCamera)
{
  std::array<double, 2> pointInImage{};

  const auto z = homography[3 * 2 + 0] * pointInCamera[0] +
                 homography[3 * 2 + 1] * pointInCamera[1] + homography[3 * 2 + 2];

  for (std::size_t index = 0; index < 2; ++index)
  {
    pointInImage[index] = (homography[3 * index + 0] * pointInCamera[0] +
                           homography[3 * index + 1] * pointInCamera[1] + homography[3 * index + 2]) /
                          z;
  }

  return pointInImage;
}

