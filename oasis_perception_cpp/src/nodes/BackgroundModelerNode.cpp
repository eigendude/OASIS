/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerNode.h"

#include "image/BackgroundModelerABL.h"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace OASIS;

namespace
{
// Published topics
constexpr const char* BACKGROUND_TOPIC = "background";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Parameters
constexpr const char* SYSTEM_ID_PARAMETER = "system_id";
constexpr const char* DEFAULT_SYSTEM_ID = "";
} // namespace

BackgroundModelerNode::BackgroundModelerNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER, DEFAULT_SYSTEM_ID);
}

BackgroundModelerNode::~BackgroundModelerNode() = default;

bool BackgroundModelerNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting background modeler...");

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
  const std::string backgroundTopic = systemId + "_" + BACKGROUND_TOPIC;

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
