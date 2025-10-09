/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <vector>

#include <oasis_msgs/msg/i2_c_imu.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ORB_SLAM3
{
class System;

namespace IMU
{
class Point;
}
} // namespace ORB_SLAM3

namespace rclcpp
{
class Logger;
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{

class MonocularInertialSlam
{
public:
  MonocularInertialSlam(rclcpp::Node& node);
  ~MonocularInertialSlam();

  // Lifecycle interface
  bool Initialize();
  void Deinitialize();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void ImuCallback(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg);

private:
  // ROS parameters
  std::unique_ptr<rclcpp::Logger> m_logger;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;
  std::vector<ORB_SLAM3::IMU::Point> m_imuMeasurements;
};

} // namespace SLAM
} // namespace OASIS
