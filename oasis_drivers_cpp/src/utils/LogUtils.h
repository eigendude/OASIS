/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of Oasis - https://github.com/eigendude/oasis
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <rclcpp/logger.hpp>

namespace rclcpp
{
  class Node;
}

namespace OASIS
{
namespace UTILS
{

class LogUtils
{
public:
  static rclcpp::Logger InitializeLogging(const std::shared_ptr<rclcpp::Node>& node);
};

}
}
