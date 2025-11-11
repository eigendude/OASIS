/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundSubtractorNode.h"

#include "background/BackgroundSubtractorASBL.h"

#include <string_view>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
// Published topics
constexpr std::string_view FOREGROUND_TOPIC = "foreground";
constexpr std::string_view SUBTRACTED_TOPIC = "subtracted";

// Subscribed topics
constexpr std::string_view IMAGE_TOPIC = "image";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";
} // namespace

BackgroundSubtractorNode::BackgroundSubtractorNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
}

BackgroundSubtractorNode::~BackgroundSubtractorNode() = default;

bool BackgroundSubtractorNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting background subtractor...");

  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId))
  {
    RCLCPP_ERROR(m_node.get_logger(), "Missing system ID parameter '%s'",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(m_node.get_logger(), "System ID parameter '%s' is empty",
                 SYSTEM_ID_PARAMETER.data());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string foregroundTopic = systemId;
  foregroundTopic.push_back('_');
  foregroundTopic.append(FOREGROUND_TOPIC);

  std::string subtractedTopic = systemId;
  subtractedTopic.push_back('_');
  subtractedTopic.append(SUBTRACTED_TOPIC);

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
