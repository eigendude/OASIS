/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "BackgroundModelerNode.h"

namespace
{
constexpr const char* NODE_NAME = "background_substractor";
}

using namespace OASIS;
using namespace ROS;

BackgroundModelerNode::BackgroundModelerNode() : rclcpp::Node(NODE_NAME)
{
}

BackgroundModelerNode::~BackgroundModelerNode() = default;
