/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
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

} // namespace UTILS
} // namespace OASIS
