/*
 *  Copyright (C) 2025-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "HelloWorldNode.h"

#include "pose/HelloWorld.h"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace oasis_perception;

HelloWorldNode::HelloWorldNode(rclcpp::Node& node)
  : m_node(node), m_helloWorld(std::make_unique<HelloWorld>())
{
}

HelloWorldNode::~HelloWorldNode() = default;

bool HelloWorldNode::Start()
{
  m_helloWorld->Initialize();

  const mediapipe_facade::HelloWorldResult result = m_helloWorld->Run();
  if (!result.success)
  {
    RCLCPP_ERROR(m_node.get_logger(), "HelloWorld facade failed: %s", result.message.c_str());
    return false;
  }

  RCLCPP_INFO(m_node.get_logger(), "HelloWorld facade succeeded with %d packets: %s",
              result.outputPacketCount, result.message.c_str());

  return true;
}

void HelloWorldNode::Stop()
{
  // TODO
}
