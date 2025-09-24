/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/BackgroundSubtractorNode.h"

#include <bgslibrary/algorithms/AdaptiveSelectiveBackgroundLearning.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/transport_hints.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;

namespace OASIS
{
namespace
{
constexpr const char* IMAGE_TOPIC = "image";
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* SUBTRACTED_TOPIC = "subtracted";

constexpr const char* ZONE_ID_PARAMETER = "zone_id";
constexpr const char* DEFAULT_ZONE_ID = "";
} // namespace

BackgroundSubtractorNode::BackgroundSubtractorNode(rclcpp::Node& node)
  : m_node(node),
    m_imgPublisherForeground(std::make_unique<image_transport::Publisher>()),
    m_imgPublisherSubtracted(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackage(std::make_unique<bgslibrary::algorithms::AdaptiveSelectiveBackgroundLearning>())
{
  m_node.declare_parameter<std::string>(ZONE_ID_PARAMETER, DEFAULT_ZONE_ID);
}

BackgroundSubtractorNode::~BackgroundSubtractorNode() = default;

bool BackgroundSubtractorNode::Initialize()
{
  if (!m_node.get_parameter(ZONE_ID_PARAMETER, m_zoneId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing zone ID parameter '%s'", ZONE_ID_PARAMETER);
    return false;
  }

  if (m_zoneId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "Zone ID parameter '%s' is empty", ZONE_ID_PARAMETER);
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "Zone ID: %s", m_zoneId.c_str());

  const std::string imageTopic = m_zoneId + "_" + IMAGE_TOPIC;
  const std::string foregroundTopic = m_zoneId + "_" + FOREGROUND_TOPIC;
  const std::string subtractedTopic = m_zoneId + "_" + SUBTRACTED_TOPIC;

  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Foreground topic: %s", foregroundTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Subtracted topic: %s", subtractedTopic.c_str());

  *m_imgPublisherForeground = image_transport::create_publisher(&m_node, foregroundTopic);
  *m_imgPublisherSubtracted = image_transport::create_publisher(&m_node, subtractedTopic);
  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic, [this](const auto& msg) { ReceiveImage(msg); }, "compressed");

  RCLCPP_INFO(m_node.get_logger(), "Started background subtractor");

  return true;
}

void BackgroundSubtractorNode::Deinitialize()
{
  m_imgSubscriber->shutdown();
  m_imgPublisherForeground->shutdown();
  m_imgPublisherSubtracted->shutdown();
}

void BackgroundSubtractorNode::ReceiveImage(
    const std::shared_ptr<const sensor_msgs::msg::Image>& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(m_node.get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage imageMask(cv_ptr->header, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImage imageBackgroundModel(cv_ptr->header, cv_ptr->encoding);
  cv_bridge::CvImage imageSubtracted(cv_ptr->header, cv_ptr->encoding);

  m_bgsPackage->process(cv_ptr->image, imageMask.image, imageBackgroundModel.image);

  if (!imageMask.image.empty())
  {
    m_imgPublisherForeground->publish(imageMask.toImageMsg());

    cv_ptr->image.copyTo(imageSubtracted.image, imageMask.image);
    m_imgPublisherSubtracted->publish(imageSubtracted.toImageMsg());
  }
}

} // namespace OASIS
