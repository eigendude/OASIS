/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerNode.h"

#include "image/BackgroundModelerABL.h"

#include <rclcpp/node.hpp>
#include <rcutils/logging_macros.h>

using namespace OASIS;

namespace
{
// Published topics
constexpr const char* BACKGROUND_TOPIC = "background";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image";

// Parameters
constexpr const char* ZONE_ID_PARAMETER = "zone_id";
constexpr const char* DEFAULT_ZONE_ID = "";
} // namespace

BackgroundModelerNode::BackgroundModelerNode(rclcpp::Node& node) : m_node(node)
{
  m_node.declare_parameter<std::string>(ZONE_ID_PARAMETER, DEFAULT_ZONE_ID);
}

BackgroundModelerNode::~BackgroundModelerNode() = default;

bool BackgroundModelerNode::Initialize()
{
  RCLCPP_INFO(m_node.get_logger(), "Starting background modeler...");

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
  const std::string backgroundTopic = zoneId + "_" + BACKGROUND_TOPIC;

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
