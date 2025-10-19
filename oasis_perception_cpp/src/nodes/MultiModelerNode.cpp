/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MultiModelerNode.h"

#include "image/MultiModeler.h"

#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
// Published topics
constexpr std::string_view FOREGROUND_TOPIC = "foreground";
constexpr std::string_view BACKGROUND_TOPIC = "background";
constexpr std::string_view SUBTRACTED_TOPIC = "subtracted";

// Subscribed topics
constexpr std::string_view IMAGE_TOPIC = "image";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";
} // namespace

MultiModelerNode::MultiModelerNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID);
}

MultiModelerNode::~MultiModelerNode() = default;

bool MultiModelerNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting multi modeler...");

  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId))
  {
    RCLCPP_ERROR(
        m_node.get_logger(), "Missing system ID parameter '%s'", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  if (systemId.empty())
  {
    RCLCPP_ERROR(
        m_node.get_logger(), "System ID parameter '%s' is empty", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "System ID: %s", systemId.c_str());

  std::string imageTopic = systemId;
  imageTopic.push_back('_');
  imageTopic.append(IMAGE_TOPIC);

  std::string foregroundTopic = systemId;
  foregroundTopic.push_back('_');
  foregroundTopic.append(FOREGROUND_TOPIC);

  std::string backgroundTopic = systemId;
  backgroundTopic.push_back('_');
  backgroundTopic.append(BACKGROUND_TOPIC);

  std::string subtractedTopic = systemId;
  subtractedTopic.push_back('_');
  subtractedTopic.append(SUBTRACTED_TOPIC);

  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Foreground topic: %s", foregroundTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Background topic: %s", backgroundTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Subtracted topic: %s", subtractedTopic.c_str());

  auto nodeShared = m_node.shared_from_this();
  m_multiModeler = std::make_unique<IMAGE::MultiModeler>(nodeShared, imageTopic, foregroundTopic,
                                                         backgroundTopic, subtractedTopic);

  RCLCPP_INFO(m_node.get_logger(), "Started multi modeler");

  return true;
}

void MultiModelerNode::Deinitialize()
{
  m_multiModeler.reset();
}
