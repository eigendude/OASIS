/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

#include <rclcpp/node.hpp>

namespace OASIS
{
namespace ROS
{

class BackgroundModelerNode : public rclcpp::Node
{
public:
  BackgroundModelerNode(const std::string& nodeName);
  ~BackgroundModelerNode() override;
};

} // namespace ROS
} // namespace OASIS
