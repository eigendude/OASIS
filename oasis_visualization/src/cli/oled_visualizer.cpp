/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/oled_visualizer_node.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<OASIS::Visualization::OledVisualizerNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(rclcpp::get_logger("oled_visualizer"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
