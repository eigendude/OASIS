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
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;

namespace OASIS
{
// Default node name
constexpr const char* NODE_NAME = "background_modeler";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Published topics
constexpr const char* BACKGROUND_TOPIC = "background";
} // namespace OASIS

BackgroundModelerNode::BackgroundModelerNode()
  : rclcpp::Node(NODE_NAME),
    m_imgPublisherBackground(std::make_unique<image_transport::Publisher>()),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>()),
    m_bgsPackage(std::make_unique<bgslibrary::algorithms::AdaptiveBackgroundLearning>())
{
}

BackgroundModelerNode::~BackgroundModelerNode() = default;

void BackgroundModelerNode::Initialize()
{
  m_imgTransport = std::make_unique<image_transport::ImageTransport>(shared_from_this());

  RCLCPP_INFO(get_logger(), "Image topic: %s", IMAGE_TOPIC);
  RCLCPP_INFO(get_logger(), "Background topic: %s", BACKGROUND_TOPIC);

  auto transportHints = image_transport::TransportHints(this, "compressed");

  *m_imgSubscriber = m_imgTransport->subscribe(IMAGE_TOPIC, 1, &BackgroundModelerNode::ReceiveImage,
                                               this, &transportHints);

  *m_imgPublisherBackground = m_imgTransport->advertise(BACKGROUND_TOPIC, 10);

  RCLCPP_INFO(get_logger(), "Started background modeler");
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
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImage imageMask(cv_ptr->header, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImage imageBackgroundModel(cv_ptr->header, cv_ptr->encoding);

  m_bgsPackage->process(cv_ptr->image, imageMask.image, imageBackgroundModel.image);

  if (!imageBackgroundModel.image.empty())
    m_imgPublisherBackground->publish(imageBackgroundModel.toImageMsg());
}
