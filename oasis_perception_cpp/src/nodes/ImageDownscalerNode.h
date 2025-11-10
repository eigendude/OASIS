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
class ImageDownscaler;
} // namespace IMAGE

class ImageDownscalerNode
{
public:
  explicit ImageDownscalerNode(rclcpp::Node& node);
  ~ImageDownscalerNode();

  bool Initialize();
  void Deinitialize();

private:
  rclcpp::Node& m_node;
  std::unique_ptr<IMAGE::ImageDownscaler> m_downscaler;
};

} // namespace OASIS

