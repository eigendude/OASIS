/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerNode.h"

#include "background/BackgroundModelerABL.h"

#include <string_view>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
// Published topics
constexpr std::string_view BACKGROUND_TOPIC = "background";

// Subscribed topics
constexpr std::string_view IMAGE_TOPIC = "image";

// Parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";
} // namespace

BackgroundModelerNode::BackgroundModelerNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
}

BackgroundModelerNode::~BackgroundModelerNode() = default;

bool BackgroundModelerNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting background modeler...");

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

  std::string backgroundTopic = systemId;
  backgroundTopic.push_back('_');
  backgroundTopic.append(BACKGROUND_TOPIC);

  RCLCPP_INFO(m_node.get_logger(), "Image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_node.get_logger(), "Background topic: %s", backgroundTopic.c_str());

  m_backgroundModeler =
      std::make_unique<IMAGE::BackgroundModelerABL>(m_node, imageTopic, backgroundTopic);

  RCLCPP_INFO(m_node.get_logger(), "Started background modeler");

  return true;
}

void BackgroundModelerNode::Deinitialize()
{
  m_backgroundModeler.reset();
}
