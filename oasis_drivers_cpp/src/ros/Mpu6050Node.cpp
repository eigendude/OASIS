/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050Node.h"

#include <algorithm>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr const char* NODE_NAME = "mpu6050_imu_driver";
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* FRAME_ID = "imu_link";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
} // namespace

using namespace OASIS::ROS;
using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);

  m_publisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(10));

  const double publishRateHz = get_parameter("publish_rate_hz").as_double();
  const double clampedRate = std::max(publishRateHz, 1.0);
  const auto publishPeriod = std::chrono::duration<double>(1.0 / clampedRate);

  m_timer = create_wall_timer(publishPeriod, std::bind(&Mpu6050Node::PublishImu, this));
}

void Mpu6050Node::PublishImu()
{
  auto imuMsg = sensor_msgs::msg::Imu();

  std_msgs::msg::Header& header = imuMsg.header;
  header.stamp = get_clock()->now();
  header.frame_id = FRAME_ID;

  geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  geometry_msgs::msg::Vector3& linearAceleration = imuMsg.linear_acceleration;

  linearAceleration.x = 0.0;
  linearAceleration.y = 0.0;
  linearAceleration.z = 0.0;

  angularVelocity.x = 0.0;
  angularVelocity.y = 0.0;
  angularVelocity.z = 0.0;

  m_publisher->publish(imuMsg);
}
