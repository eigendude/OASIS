/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <oasis_msgs/msg/i2_c_imu.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_transport
{
class ImageTransport;
class Publisher;
class Subscriber;
} // namespace image_transport

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
class Node;
} // namespace rclcpp

namespace OASIS
{
namespace SLAM
{

class MonocularInertialSlam
{
public:
  MonocularInertialSlam(std::shared_ptr<rclcpp::Node> node,
                        const std::string& imageTopic,
                        const std::string& imuTopic);
  ~MonocularInertialSlam();

  // ROS interface
  void ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void ImuCallback(const oasis_msgs::msg::I2CImu::ConstSharedPtr& msg);

private:
  // Logging parameters
  rclcpp::Logger m_logger;

  // ROS parameters
  std::unique_ptr<image_transport::ImageTransport> m_imgTransport;
  std::unique_ptr<image_transport::Subscriber> m_imgSubscriber;
  std::shared_ptr<rclcpp::Subscription<oasis_msgs::msg::I2CImu>> m_imuSubscriber;

  // ORB-SLAM3 system
  std::unique_ptr<ORB_SLAM3::System> m_slam;
  std::vector<ORB_SLAM3::IMU::Point> m_imuMeasurements;
};

} // namespace SLAM
} // namespace OASIS
