/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/BackgroundModelerNode.h"

#include "utils/NetworkUtils.h"

#include <bgslibrary/algorithms/AdaptiveBackgroundLearning.h>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;

namespace OASIS
{
// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Published topics
constexpr const char* BACKGROUND_TOPIC = "background";

// Parameters
constexpr const char* ZONE_ID_PARAMETER = "zone_id";
constexpr const char* DEFAULT_ZONE_ID = "";
} // namespace OASIS

BackgroundModelerNode::BackgroundModelerNode(rclcpp::Node& node)
  : m_node(node),
    m_imgPublisherBackground(),
    m_imgSubscriber(),
    m_bgsPackage(std::make_unique<bgslibrary::algorithms::AdaptiveBackgroundLearning>())
{
}

BackgroundModelerNode::~BackgroundModelerNode() = default;

bool BackgroundModelerNode::Start()
{
  try
  {
    m_nodeShared = m_node.shared_from_this();
  }
  catch (const std::bad_weak_ptr&)
  {
    RCLCPP_ERROR(m_node.get_logger(), "BackgroundModelerNode requires a shared_ptr-managed node");
    return false;
  }

  m_zoneId = m_node.declare_parameter<std::string>(ZONE_ID_PARAMETER, DEFAULT_ZONE_ID);

  if (m_zoneId.empty())
    RCLCPP_WARN(m_node.get_logger(), "Zone ID parameter '%s' is empty", ZONE_ID_PARAMETER);
  else
    RCLCPP_INFO(m_node.get_logger(), "Zone ID: %s", m_zoneId.c_str());

  m_imgTransport = std::make_unique<image_transport::ImageTransport>(m_nodeShared);

  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", IMAGE_TOPIC);
  RCLCPP_INFO(m_node.get_logger(), "Background topic: %s", BACKGROUND_TOPIC);

  image_transport::TransportHints transportHints(m_nodeShared.get(), "compressed");

  m_imgSubscriber = m_imgTransport->subscribe(
    IMAGE_TOPIC,
    1,
    std::bind(&BackgroundModelerNode::ReceiveImage, this, std::placeholders::_1),
    transportHints);

  m_imgPublisherBackground = m_imgTransport->advertise(BACKGROUND_TOPIC, 10);

  RCLCPP_INFO(m_node.get_logger(), "Started background modeler");

  return true;
}

void BackgroundModelerNode::Stop()
{
  if (m_imgTransport)
  {
    if (m_imgSubscriber)
      m_imgSubscriber.shutdown();

    if (m_imgPublisherBackground)
      m_imgPublisherBackground.shutdown();

    m_imgTransport.reset();
  }
}

void BackgroundModelerNode::ReceiveImage(const std::shared_ptr<const sensor_msgs::msg::Image>& msg)
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

  m_bgsPackage->process(cv_ptr->image, imageMask.image, imageBackgroundModel.image);

  if (!imageBackgroundModel.image.empty())
    m_imgPublisherBackground.publish(imageBackgroundModel.toImageMsg());
}
