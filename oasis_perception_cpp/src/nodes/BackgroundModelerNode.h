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
} // namespace rclcpp

namespace OASIS
{
namespace IMAGE
{
class BackgroundModelerABL;
}

class BackgroundModelerNode
{
public:
  explicit BackgroundModelerNode(rclcpp::Node& node);
  ~BackgroundModelerNode();

  bool Initialize();
  void Deinitialize();

private:
  // Construction parameters
  rclcpp::Node& m_node;

  // Image parameters
  std::unique_ptr<IMAGE::BackgroundModelerABL> m_backgroundModeler;
};

} // namespace OASIS
