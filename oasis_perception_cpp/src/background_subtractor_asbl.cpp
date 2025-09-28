/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "image/BackgroundSubtractorASBL.h"
#include "utils/NetworkUtils.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace OASIS
{
constexpr const char* ROS_NAMESPACE = "oasis"; // TODO

constexpr const char* BGS_ASBL_NODE_NAME = "background_substractor_asbl";

// TODO: Hardware configuration
constexpr const char* VIDEO_MACHINE = "livingroom";

// Subscribed topics
constexpr const char* IMAGE_TOPIC = "image_raw";

// Published topics
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* SUBTRACTED_TOPIC = "subtracted";
} // namespace OASIS

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Get the hostname
  const std::string appendHostname = "_" + OASIS::UTILS::NetworkUtils::GetHostName();

  // Create node
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>(BGS_ASBL_NODE_NAME + appendHostname);

  // Create topics
  const std::string topicBase = std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE + "/";

  {
    OASIS::IMAGE::BackgroundSubtractorASBL backgroundModeler(
        node, topicBase + IMAGE_TOPIC, topicBase + FOREGROUND_TOPIC, topicBase + SUBTRACTED_TOPIC);

    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
