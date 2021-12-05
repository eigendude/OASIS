/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <rclcpp/node.hpp>

namespace OASIS
{
namespace ROS
{

class LedServerNode : public rclcpp::Node
{
public:
  LedServerNode();
  ~LedServerNode() override;
};

} // namespace ROS
} // namespace OASIS
