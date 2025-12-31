/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050Node.h"

#include "imu/Mpu6050ImuUtils.h"

#include <chrono>
#include <cmath>
#include <cstdint>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>

using namespace OASIS::ROS;
using namespace std::chrono_literals;

namespace
{

// Default node name
constexpr const char* NODE_NAME = "mpu6050_imu_driver";
constexpr double GRAVITY_MPS2 = 9.80665;

// ROS topics
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* IMU_RAW_TOPIC = "imu_raw";
constexpr const char* IMU_TEMPERATURE_TOPIC = "imu_temperature";

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double kRollPitchVariance = 0.0076;
constexpr double kYawVariance = 1e6;
constexpr double kPi = 3.141592653589793;
constexpr double kTwoPi = 2.0 * kPi;
constexpr bool kLogStationaryTransitions = true;

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

  // Log WHO_AM_I
  const uint8_t who = m_mpu6050->getDeviceID();
  RCLCPP_INFO(get_logger(), "MPU6050 WHO_AM_I / device ID: 0x%02X", who);

  // Ensure full-scale ranges match our scaling assumptions
  m_mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  m_mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  const uint8_t accelRange = m_mpu6050->getFullScaleAccelRange();
  const uint8_t gyroRange = m_mpu6050->getFullScaleGyroRange();

  // Initialize IMU state based on actual configuration
  const double accelScale = IMU::Mpu6050ImuUtils::AccelScaleFromRange(accelRange);
  const double gyroScale = IMU::Mpu6050ImuUtils::GyroScaleFromRange(gyroRange);

  m_imuProcessor.SetAccelScale(accelScale);
  m_imuProcessor.SetGyroScale(gyroScale);

  RCLCPP_INFO(get_logger(), "MPU6050 full-scale ranges set (accel=%u, gyro=%u)",
              static_cast<unsigned>(accelRange), static_cast<unsigned>(gyroRange));

  // Initialize publishers
  m_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS{1});
  m_imuRawPublisher = create_publisher<sensor_msgs::msg::Imu>(IMU_RAW_TOPIC, rclcpp::QoS{1});
  m_imuTemperaturePublisher =
      create_publisher<sensor_msgs::msg::Temperature>(IMU_TEMPERATURE_TOPIC, rclcpp::QoS{1});

  // Initialize timers
  m_timer = create_wall_timer(m_publishPeriod, std::bind(&Mpu6050Node::PublishImu, this));

  return true;
}

void Mpu6050Node::Deinitialize()
{
  // TODO
}

void Mpu6050Node::PublishImu()
{
  if (!m_mpu6050)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MPU6050 not connected");
    return;
  }

  const bool dataReady = m_mpu6050->getIntDataReadyStatus();
  if (!dataReady)
  {
    // TODO: Log error and return
  }

  // Capture motion
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  m_mpu6050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Create header for published messages
  std_msgs::msg::Header headerMsg;
  headerMsg.stamp = get_clock()->now();
  headerMsg.frame_id = FRAME_ID;

  // Process motion
  const auto processed = m_imuProcessor.ProcessRaw(ax, ay, az, gx, gy, gz);

  const double raw_accel_norm_lsb = std::sqrt(static_cast<double>(ax) * static_cast<double>(ax) +
                                              static_cast<double>(ay) * static_cast<double>(ay) +
                                              static_cast<double>(az) * static_cast<double>(az));
  const double accel_scale = m_imuProcessor.GetAccelScale();
  const double raw_accel_norm_mps2 = raw_accel_norm_lsb * accel_scale;

  // Publish motion
  sensor_msgs::msg::Imu imuMsg;
  sensor_msgs::msg::Imu imuRawMsg;

  auto fillImuMsg = [&](sensor_msgs::msg::Imu& imuMsg)
  {
    imuMsg.header = headerMsg;

    geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
    geometry_msgs::msg::Vector3& linearAcceleration = imuMsg.linear_acceleration;

    linearAcceleration.x = processed.accel_raw_mps2[0];
    linearAcceleration.y = processed.accel_raw_mps2[1];
    linearAcceleration.z = processed.accel_raw_mps2[2];

    angularVelocity.x = processed.gyro_rads[0];
    angularVelocity.y = processed.gyro_rads[1];
    angularVelocity.z = processed.gyro_rads[2];

    imuMsg.angular_velocity_covariance.fill(0.0);
    imuMsg.linear_acceleration_covariance.fill(0.0);
  };

  // TODO: The IMU messages will be different
  fillImuMsg(imuMsg);
  fillImuMsg(imuRawMsg);

  m_imuPublisher->publish(imuMsg);
  m_imuRawPublisher->publish(imuRawMsg);

  // Capture temperature
  const int16_t tempRaw = m_mpu6050->getTemperature();

  // Process temperature
  // MPU6050 datasheet formula: Temp(°C) = (TEMP_OUT / 340) + 36.53
  const double tempC = (static_cast<double>(tempRaw) / 340.0) + 36.53;

  // Publish temperature
  sensor_msgs::msg::Temperature temperatureMsg;

  temperatureMsg.header = headerMsg;
  temperatureMsg.temperature = tempC;
  temperatureMsg.variance = 0.0;

  m_imuTemperaturePublisher->publish(temperatureMsg);

  // Log data
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "IMU: |a|=%.3f m/s^2, temp=%.2fC",
                       raw_accel_norm_mps2, tempC);
}
