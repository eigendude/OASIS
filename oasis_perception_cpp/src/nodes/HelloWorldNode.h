/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */
#pragma once

#include <memory>

namespace rclcpp
{
class Node;
}

namespace oasis_perception
{
class HelloWorld;

class HelloWorldNode
{
public:
  HelloWorldNode(rclcpp::Node& node);
  ~HelloWorldNode();

  bool Start();
  void Stop();

private:
  // Construction parameters
  rclcpp::Node& m_node;

  // Hello world instance
  std::unique_ptr<HelloWorld> m_helloWorld;
};
} // namespace oasis_perception
