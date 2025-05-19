/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "HelloWorldNode.h"

#include "utils/HelloWorld.h"

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

  m_helloWorld->Run();

  return true;
}

void HelloWorldNode::Stop()
{
  // TODO
}
