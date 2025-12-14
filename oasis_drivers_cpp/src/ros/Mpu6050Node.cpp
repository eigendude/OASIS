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
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr const char* NODE_NAME = "mpu6050_imu_driver";
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* FRAME_ID = "imu_link";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double GRAVITY = 9.80665; // m/s^2
constexpr double ACCEL_SCALE = GRAVITY / 16384.0; // +/-2g full scale
constexpr double GYRO_SCALE = (M_PI / 180.0) / 131.0; // +/-250 deg/s full scale
} // namespace

using namespace OASIS::ROS;
using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));

  m_publisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(10));

  const double publishRateHz = get_parameter("publish_rate_hz").as_double();
  const double clampedRate = std::max(publishRateHz, 1.0);
  const auto publishPeriod = std::chrono::duration<double>(1.0 / clampedRate);

  const std::string i2cDevice = get_parameter("i2c_device").as_string();
  I2Cdev::initialize(i2cDevice.c_str());

  m_mpu6050 = std::make_unique<MPU6050>();
  m_mpu6050->initialize();

  m_isConnected = m_mpu6050->testConnection();
  if (!m_isConnected)
  {
    RCLCPP_ERROR(get_logger(), "Failed to connect to MPU6050 on %s", i2cDevice.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Connected to MPU6050 on %s", i2cDevice.c_str());
  }

  m_accelScale = ACCEL_SCALE;
  m_gyroScale = GYRO_SCALE;

  m_timer = create_wall_timer(publishPeriod, std::bind(&Mpu6050Node::PublishImu, this));
}

void Mpu6050Node::PublishImu()
{
  if (!m_isConnected)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MPU6050 not connected");
    return;
  }

  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;

  m_mpu6050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  auto imuMsg = sensor_msgs::msg::Imu();

  std_msgs::msg::Header& header = imuMsg.header;
  header.stamp = get_clock()->now();
  header.frame_id = FRAME_ID;

  geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  geometry_msgs::msg::Vector3& linearAceleration = imuMsg.linear_acceleration;

  linearAceleration.x = ax * m_accelScale;
  linearAceleration.y = ay * m_accelScale;
  linearAceleration.z = az * m_accelScale;

  angularVelocity.x = gx * m_gyroScale;
  angularVelocity.y = gy * m_gyroScale;
  angularVelocity.z = gz * m_gyroScale;

  imuMsg.orientation.w = 1.0;
  imuMsg.orientation.x = 0.0;
  imuMsg.orientation.y = 0.0;
  imuMsg.orientation.z = 0.0;

  m_publisher->publish(imuMsg);
}
