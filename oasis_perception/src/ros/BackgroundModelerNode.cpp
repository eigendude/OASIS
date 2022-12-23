/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerNode.h"

using namespace OASIS;
using namespace ROS;

BackgroundModelerNode::BackgroundModelerNode(const std::string& nodeName) : rclcpp::Node(nodeName)
{
}

BackgroundModelerNode::~BackgroundModelerNode() = default;
