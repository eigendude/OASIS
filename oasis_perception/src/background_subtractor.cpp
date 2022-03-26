/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "image/BackgroundModeler.h"
#include "ros/BackgroundModelerNode.h"

#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace OASIS
{
// TODO: Hardware configuration
constexpr const char* VIDEO_MACHINE_LENOVO = "lenovo";
constexpr const char* VIDEO_MACHINE_NETBOOK = "netbook";
} // namespace OASIS

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create node
  std::shared_ptr<rclcpp::Node> node = std::make_shared<OASIS::ROS::BackgroundModelerNode>();

  {
    OASIS::IMAGE::BackgroundModeler backgroundModelerLenovo(node, VIDEO_MACHINE_LENOVO);
    OASIS::IMAGE::BackgroundModeler backgroundModelerNetbook(node, VIDEO_MACHINE_NETBOOK);
    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
