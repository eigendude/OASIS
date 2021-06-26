/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "LedServerNode.h"

namespace
{
  constexpr const char* NODE_NAME = "led_server";
}

using namespace OASIS;
using namespace ROS;

LedServerNode::LedServerNode() : rclcpp::Node(NODE_NAME)
{
}

LedServerNode::~LedServerNode() = default;
