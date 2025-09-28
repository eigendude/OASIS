/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundSubtractorNode.h"

#include "image/BackgroundSubtractorASBL.h"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
// Published topics
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* SUBTRACTED_TOPIC = "subtracted";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Parameters
constexpr const char* ZONE_ID_PARAMETER = "zone_id";
constexpr const char* DEFAULT_ZONE_ID = "";
} // namespace

BackgroundSubtractorNode::BackgroundSubtractorNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(ZONE_ID_PARAMETER, DEFAULT_ZONE_ID);
}

BackgroundSubtractorNode::~BackgroundSubtractorNode() = default;

bool BackgroundSubtractorNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting background subtractor...");

  std::string zoneId;
  if (!m_node.get_parameter(ZONE_ID_PARAMETER, zoneId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing zone ID parameter '%s'", ZONE_ID_PARAMETER);
    return false;
  }

  if (zoneId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "Zone ID parameter '%s' is empty", ZONE_ID_PARAMETER);
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "Zone ID: %s", zoneId.c_str());

  const std::string imageTopic = zoneId + "_" + IMAGE_TOPIC;
  const std::string foregroundTopic = zoneId + "_" + FOREGROUND_TOPIC;
  const std::string subtractedTopic = zoneId + "_" + SUBTRACTED_TOPIC;

  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Foreground topic: %s", foregroundTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Subtracted topic: %s", subtractedTopic.c_str());

  m_backgroundSubtractor = std::make_unique<IMAGE::BackgroundSubtractorASBL>(
      m_node, imageTopic, foregroundTopic, subtractedTopic);

  RCLCPP_INFO(m_node.get_logger(), "Started background subtractor");

  return true;
}

void BackgroundSubtractorNode::Deinitialize()
{
  m_backgroundSubtractor.reset();
}
