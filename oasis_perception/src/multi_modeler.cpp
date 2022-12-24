/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "image/MultiModeler.h"
#include "utils/NetworkUtils.h"

#include <memory>

#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace OASIS
{
constexpr const char* ROS_NAMESPACE = "oasis"; // TODO

constexpr const char* MULTI_MODELER_NODE_NAME = "multi_modeler";

// TODO: Hardware configuration
constexpr const char* VIDEO_MACHINE_KINECT2 = "kinect2";

// Subscribed topics
constexpr const char* IMAGE_TOPIC_KINECT2 = "hd/image_color";

// Published topics
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* BACKGROUND_TOPIC = "background";
constexpr const char* SUBTRACTED_TOPIC = "subtracted";
} // namespace OASIS

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Get the hostname
  const std::string appendHostname = "_" + OASIS::UTILS::NetworkUtils::GetHostName();

  // Create node
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>(MULTI_MODELER_NODE_NAME + appendHostname);

  // Create topics
  const std::string kinectTopicBase =
      std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE_KINECT2 + "/";

  {
    OASIS::IMAGE::MultiModeler multiModelerKinect2(
        node, kinectTopicBase + IMAGE_TOPIC_KINECT2, kinectTopicBase + FOREGROUND_TOPIC,
        kinectTopicBase + BACKGROUND_TOPIC, kinectTopicBase + SUBTRACTED_TOPIC);

    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
