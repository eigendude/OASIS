/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlamNode.h"

#include "slam/MonocularSlam.h"

#include <algorithm>
#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace ROS;

namespace
{
// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Parameters
constexpr const char* SYSTEM_ID_PARAMETER = "system_id";
constexpr const char* DEFAULT_SYSTEM_ID = "";
} // namespace

MonocularSlamNode::MonocularSlamNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(std::make_unique<rclcpp::Logger>(node.get_logger())),
    m_imgSubscriber(std::make_unique<image_transport::Subscriber>())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER, DEFAULT_SYSTEM_ID);
}

MonocularSlamNode::~MonocularSlamNode() = default;

bool MonocularSlamNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER, systemId))
  {
    RCLCPP_ERROR(*m_logger, "Missing system ID parameter '%s'", SYSTEM_ID_PARAMETER);
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(*m_logger, "System ID parameter '%s' is empty", SYSTEM_ID_PARAMETER);
    return false;
  }

  const std::string imageTopic = systemId + "_" + IMAGE_TOPIC;

  RCLCPP_INFO(*m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(*m_logger, "Image topic: %s", imageTopic.c_str());

  //*m_flowPublisher = image_transport::create_publisher(&m_node, flowTopic);

  *m_imgSubscriber = image_transport::create_subscription(
      &m_node, imageTopic,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { OnImage(msg); }, "compressed");

  m_monocularSlam = std::make_unique<SLAM::MonocularSlam>(m_node);
  if (!m_monocularSlam->Initialize())
  {
    RCLCPP_ERROR(*m_logger, "Failed to initialize monocular SLAM");
    return false;
  }

  RCLCPP_INFO(*m_logger, "Started monocular SLAM");

  return true;
}

void MonocularSlamNode::Deinitialize()
{
  m_imgSubscriber->shutdown();

  m_monocularSlam.reset();
}

void MonocularSlamNode::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (m_monocularSlam)
    m_monocularSlam->ReceiveImage(msg);
}
