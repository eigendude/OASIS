/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "slam/MonocularInertialSlam.h"
#include "utils/NetworkUtils.h"

#include <memory>

#include <System.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace OASIS;

namespace OASIS
{
constexpr const char* ROS_NAMESPACE = "oasis"; // TODO

constexpr const char* MONOCULAR_INERTIAL_SLAM_NODE_NAME = "monocular_inertial_slam";

// TODO: Hardware configuration
constexpr const char* VIDEO_MACHINE_KINECT2 = "kinect2";

// Subscribed topics
constexpr const char* IMAGE_TOPIC_KINECT2 = "hd/image_color";
constexpr const char* IMU_TOPIC = "i2c_imu";
} // namespace OASIS

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Get the hostname
  const std::string appendHostname = "_" + OASIS::UTILS::NetworkUtils::GetHostName();

  // Create node
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>(MONOCULAR_INERTIAL_SLAM_NODE_NAME + appendHostname);

  // Create topics
  const std::string kinectTopicBase =
      std::string("/") + ROS_NAMESPACE + "/" + VIDEO_MACHINE_KINECT2 + "/";

  {
    OASIS::SLAM::MonocularInertialSlam monocularInertialSlam(
        node, kinectTopicBase + IMAGE_TOPIC_KINECT2, IMU_TOPIC);

    rclcpp::spin(node);
  }

  rclcpp::shutdown();

  return 0;
}
