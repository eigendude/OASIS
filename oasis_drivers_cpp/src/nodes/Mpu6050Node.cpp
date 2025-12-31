/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050Node.h"

#include "imu/Mpu6050ImuUtils.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace OASIS::IMU;
using namespace OASIS::ROS;
using namespace std::chrono_literals;

namespace
{

// Default node name
constexpr const char* NODE_NAME = "mpu6050_imu_driver";

// ROS topics
constexpr const char* CONDUCTOR_STATE_TOPIC = "conductor_state";
constexpr const char* IMU_TOPIC = "imu";

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;

} // namespace

Mpu6050Node::Mpu6050Node() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);

  m_i2cDevice = get_parameter("i2c_device").as_string();

  const double publishRateHz = get_parameter("publish_rate_hz").as_double();
  const double clampedRate = std::max(publishRateHz, 1.0);
  m_publishPeriod = std::chrono::duration<double>(1.0 / clampedRate);
}

bool Mpu6050Node::Initialize()
{
  I2Cdev::initialize(m_i2cDevice.c_str());

  m_mpu6050 = std::make_unique<MPU6050>();
  m_mpu6050->initialize();

  if (!m_mpu6050->testConnection())
  {
    RCLCPP_ERROR(get_logger(), "Failed to connect to MPU6050 on %s", m_i2cDevice.c_str());
    m_mpu6050.reset();
    return false;
  }

  RCLCPP_INFO(get_logger(), "Connected to MPU6050 on %s", m_i2cDevice.c_str());

  // Ensure full-scale ranges match our scaling assumptions
  m_mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  m_mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  const uint8_t accelRange = m_mpu6050->getFullScaleAccelRange();
  const uint8_t gyroRange = m_mpu6050->getFullScaleGyroRange();

  // Initialize IMU state based on actual configuration
  const double accelScale = Mpu6050ImuUtils::AccelScaleFromRange(accelRange);
  const double gyroScale = Mpu6050ImuUtils::GyroScaleFromRange(gyroRange);

  m_imuProcessor.SetAccelScale(accelScale);
  m_imuProcessor.SetGyroScale(gyroScale);

  RCLCPP_INFO(get_logger(), "MPU6050 full-scale ranges set (accel=%u, gyro=%u)",
              static_cast<unsigned>(accelRange), static_cast<unsigned>(gyroRange));

  // Initialize publishers
  m_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS{1});
  m_conductorStateSub = create_subscription<oasis_msgs::msg::ConductorState>(
      CONDUCTOR_STATE_TOPIC, rclcpp::QoS{1},
      std::bind(&Mpu6050Node::OnConductorState, this, std::placeholders::_1));

  // Initialize timers
  m_timer = create_wall_timer(m_publishPeriod, std::bind(&Mpu6050Node::PublishImu, this));

  return true;
}

void Mpu6050Node::Deinitialize()
{
  // TODO
}

void Mpu6050Node::OnConductorState(const oasis_msgs::msg::ConductorState& msg)
{
  m_dutyCycleInput = msg.duty_cycle;
}

void Mpu6050Node::PublishImu()
{
  if (!m_mpu6050)
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

  const int16_t tempRaw = m_mpu6050->getTemperature();

  const rclcpp::Time now = get_clock()->now();

  // TODO
  /*
  auto output = m_imuProcessor->Process(sample, m_dutyCycleInput);
  if (!output)
    return;
   */

  sensor_msgs::msg::Imu imuMsg;

  std_msgs::msg::Header& header = imuMsg.header;
  header.stamp = now;
  header.frame_id = FRAME_ID;

  geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  geometry_msgs::msg::Vector3& linearAcceleration = imuMsg.linear_acceleration;

  // TODO
  /*
  linearAcceleration.x = output->linear_acceleration[0];
  linearAcceleration.y = output->linear_acceleration[1];
  linearAcceleration.z = output->linear_acceleration[2];

  angularVelocity.x = output->angular_velocity[0];
  angularVelocity.y = output->angular_velocity[1];
  angularVelocity.z = output->angular_velocity[2];

  imuMsg.orientation.w = output->orientation[0];
  imuMsg.orientation.x = output->orientation[1];
  imuMsg.orientation.y = output->orientation[2];
  imuMsg.orientation.z = output->orientation[3];

  imuMsg.orientation_covariance = output->orientation_covariance;
  imuMsg.angular_velocity_covariance = output->angular_velocity_covariance;
  imuMsg.linear_acceleration_covariance = output->linear_acceleration_covariance;
  */

  m_imuPublisher->publish(imuMsg);
}
