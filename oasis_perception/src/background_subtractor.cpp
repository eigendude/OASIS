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
constexpr const char* ROS_NAMESPACE = "oasis"; // TODO

// TODO: Hardware configuration
constexpr const char* VIDEO_MACHINE_KINECT2 = "kinect2";
constexpr const char* VIDEO_MACHINE_LENOVO = "lenovo";
constexpr const char* VIDEO_MACHINE_NETBOOK = "netbook";

// Subscribed topics
constexpr const char* IMAGE_TOPIC_KINECT2 = "hd/image_color";
constexpr const char* IMAGE_TOPIC_LENOVO = "image_raw";
constexpr const char* IMAGE_TOPIC_NETBOOK = "image_raw";

// Published topics
constexpr const char* FOREGROUND_TOPIC = "foreground";
constexpr const char* BACKGROUND_TOPIC = "background";
} // namespace OASIS

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create node
  std::shared_ptr<rclcpp::Node> node = std::make_shared<OASIS::ROS::BackgroundModelerNode>();

  // Create topics
  const std::string kinectTopicBase =
      std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE_KINECT2 + "/";
  const std::string lenovoTopicBase =
      std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE_LENOVO + "/";
  const std::string netbookTopicBase =
      std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE_NETBOOK + "/";

  {
    OASIS::IMAGE::BackgroundModeler backgroundModelerKinect2(
        node, kinectTopicBase + IMAGE_TOPIC_KINECT2, kinectTopicBase + FOREGROUND_TOPIC,
        kinectTopicBase + BACKGROUND_TOPIC);
    OASIS::IMAGE::BackgroundModeler backgroundModelerLenovo(
        node, lenovoTopicBase + IMAGE_TOPIC_LENOVO, lenovoTopicBase + FOREGROUND_TOPIC,
        lenovoTopicBase + BACKGROUND_TOPIC);
    OASIS::IMAGE::BackgroundModeler backgroundModelerNetbook(
        node, netbookTopicBase + IMAGE_TOPIC_NETBOOK, netbookTopicBase + FOREGROUND_TOPIC,
        netbookTopicBase + BACKGROUND_TOPIC);

    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
