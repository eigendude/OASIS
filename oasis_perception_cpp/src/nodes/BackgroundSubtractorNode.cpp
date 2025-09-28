/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundSubtractorNode.h"

#include "image/BackgroundSubtractorASBL.h"

#include <rclcpp/node.hpp>
#include <rcutils/logging_macros.h>

using namespace OASIS;

namespace
{
// Published topics
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* SUBTRACTED_TOPIC = "subtracted";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Parameters
constexpr const char* SYSTEM_ID_PARAMETER = "system_id";
constexpr const char* DEFAULT_SYSTEM_ID = "";
} // namespace

BackgroundSubtractorNode::BackgroundSubtractorNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER, DEFAULT_SYSTEM_ID);
}

BackgroundSubtractorNode::~BackgroundSubtractorNode() = default;

bool BackgroundSubtractorNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting background subtractor...");

  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER, systemId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'", SYSTEM_ID_PARAMETER);
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty", SYSTEM_ID_PARAMETER);
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());

  const std::string imageTopic = systemId + "_" + IMAGE_TOPIC;
  const std::string foregroundTopic = systemId + "_" + FOREGROUND_TOPIC;
  const std::string subtractedTopic = systemId + "_" + SUBTRACTED_TOPIC;

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
