/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/ros/Bno086NodeParams.hpp"

#include "imu/bno086/utils/Bno086TimingUtils.hpp"

#include <algorithm>
#include <string>

namespace
{
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr std::uint8_t DEFAULT_I2C_ADDRESS = 0x4B;
constexpr const char* DEFAULT_GPIO_CHIP_DEVICE = "/dev/gpiochip0";
constexpr unsigned int DEFAULT_INT_GPIO = 23;
constexpr const char* DEFAULT_FRAME_ID = "imu_link";

constexpr int DEFAULT_ALL_ZERO_BACKOFF_US = 500;
constexpr int MIN_ALL_ZERO_BACKOFF_US = 0;
constexpr int MAX_ALL_ZERO_BACKOFF_US = 10'000;
constexpr double DEFAULT_PREDICTION_HORIZON_SEC = 0.0;

constexpr double DEFAULT_ROTATION_VECTOR_RATE_HZ = 50.0;
constexpr double DEFAULT_GYRO_RATE_HZ = 50.0;
constexpr double DEFAULT_ACCELEROMETER_RATE_HZ = 100.0;
constexpr double DEFAULT_LINEAR_ACCELERATION_RATE_HZ = 50.0;
constexpr double DEFAULT_GRAVITY_RATE_HZ = 25.0;
constexpr double DEFAULT_ROTATION_VECTOR_BATCH_MS = 50.0;
constexpr double DEFAULT_GYRO_BATCH_MS = 50.0;
constexpr double DEFAULT_ACCELEROMETER_BATCH_MS = 50.0;
constexpr double DEFAULT_LINEAR_ACCELERATION_BATCH_MS = 50.0;
constexpr double DEFAULT_GRAVITY_BATCH_MS = 100.0;

constexpr int DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS = 5'000;
constexpr int MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS = 1'000;

// Maximum nearby orientation age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS = 80.0;

// Maximum nearby gyro age accepted for imu_gravity composition
constexpr double DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS = 80.0;
} // namespace

namespace OASIS::ROS
{
void DeclareBno086NodeParameters(rclcpp::Node& node)
{
  node.declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  node.declare_parameter("i2c_address", static_cast<int>(DEFAULT_I2C_ADDRESS));
  node.declare_parameter("gpio_chip_device", std::string(DEFAULT_GPIO_CHIP_DEVICE));
  node.declare_parameter("int_gpio", static_cast<int>(DEFAULT_INT_GPIO));
  node.declare_parameter("bno086_rotation_vector_rate_hz", DEFAULT_ROTATION_VECTOR_RATE_HZ);
  node.declare_parameter("bno086_gyro_rate_hz", DEFAULT_GYRO_RATE_HZ);
  node.declare_parameter("bno086_accelerometer_rate_hz", DEFAULT_ACCELEROMETER_RATE_HZ);
  node.declare_parameter("bno086_linear_acceleration_rate_hz", DEFAULT_LINEAR_ACCELERATION_RATE_HZ);
  node.declare_parameter("bno086_gravity_rate_hz", DEFAULT_GRAVITY_RATE_HZ);
  node.declare_parameter("bno086_rotation_vector_batch_ms", DEFAULT_ROTATION_VECTOR_BATCH_MS);
  node.declare_parameter("bno086_gyro_batch_ms", DEFAULT_GYRO_BATCH_MS);
  node.declare_parameter("bno086_accelerometer_batch_ms", DEFAULT_ACCELEROMETER_BATCH_MS);
  node.declare_parameter("bno086_linear_acceleration_batch_ms",
                         DEFAULT_LINEAR_ACCELERATION_BATCH_MS);
  node.declare_parameter("bno086_gravity_batch_ms", DEFAULT_GRAVITY_BATCH_MS);
  node.declare_parameter("bno086_all_zero_backoff_us", DEFAULT_ALL_ZERO_BACKOFF_US);
  node.declare_parameter("bno086_diagnostics_log_period_ms",
                         DEFAULT_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  node.declare_parameter("prediction_horizon_sec", DEFAULT_PREDICTION_HORIZON_SEC);
  node.declare_parameter("imu_gravity_max_orientation_age_ms",
                         DEFAULT_IMU_GRAVITY_MAX_ORIENTATION_AGE_MS);
  node.declare_parameter("imu_gravity_max_gyro_age_ms", DEFAULT_IMU_GRAVITY_MAX_GYRO_AGE_MS);
  node.declare_parameter("frame_id", std::string(DEFAULT_FRAME_ID));
}

Bno086NodeParams LoadBno086NodeParameters(const rclcpp::Node& node)
{
  Bno086NodeParams params;

  params.transport.i2c_device = node.get_parameter("i2c_device").as_string();
  params.transport.i2c_address =
      static_cast<std::uint8_t>(node.get_parameter("i2c_address").as_int());
  params.transport.gpio_chip_device = node.get_parameter("gpio_chip_device").as_string();
  params.transport.int_gpio = node.get_parameter("int_gpio").as_int();

  params.reports.rotation_vector_rate_hz =
      std::max(node.get_parameter("bno086_rotation_vector_rate_hz").as_double(), 1.0);
  params.reports.gyro_rate_hz =
      std::max(node.get_parameter("bno086_gyro_rate_hz").as_double(), 1.0);
  params.reports.accelerometer_rate_hz =
      std::max(node.get_parameter("bno086_accelerometer_rate_hz").as_double(), 1.0);
  params.reports.linear_acceleration_rate_hz =
      std::max(node.get_parameter("bno086_linear_acceleration_rate_hz").as_double(), 1.0);
  params.reports.gravity_rate_hz =
      std::max(node.get_parameter("bno086_gravity_rate_hz").as_double(), 1.0);
  params.reports.rotation_vector_batch_interval_us = OASIS::IMU::BNO086::MillisecondsToMicroseconds(
      node.get_parameter("bno086_rotation_vector_batch_ms").as_double());
  params.reports.gyro_batch_interval_us = OASIS::IMU::BNO086::MillisecondsToMicroseconds(
      node.get_parameter("bno086_gyro_batch_ms").as_double());
  params.reports.accelerometer_batch_interval_us = OASIS::IMU::BNO086::MillisecondsToMicroseconds(
      node.get_parameter("bno086_accelerometer_batch_ms").as_double());
  params.reports.linear_acceleration_batch_interval_us =
      OASIS::IMU::BNO086::MillisecondsToMicroseconds(
          node.get_parameter("bno086_linear_acceleration_batch_ms").as_double());
  params.reports.gravity_batch_interval_us = OASIS::IMU::BNO086::MillisecondsToMicroseconds(
      node.get_parameter("bno086_gravity_batch_ms").as_double());

  params.driver.all_zero_backoff_us =
      std::clamp(static_cast<int>(node.get_parameter("bno086_all_zero_backoff_us").as_int()),
                 MIN_ALL_ZERO_BACKOFF_US, MAX_ALL_ZERO_BACKOFF_US);
  params.driver.diagnostics_log_period_ms =
      std::max(static_cast<int>(node.get_parameter("bno086_diagnostics_log_period_ms").as_int()),
               MIN_BNO086_DIAGNOSTICS_LOG_PERIOD_MS);
  params.driver.prediction_horizon_sec =
      std::max(node.get_parameter("prediction_horizon_sec").as_double(), 0.0);
  params.driver.imu_gravity_max_orientation_age_ms =
      std::max(node.get_parameter("imu_gravity_max_orientation_age_ms").as_double(), 0.0);
  params.driver.imu_gravity_max_gyro_age_ms =
      std::max(node.get_parameter("imu_gravity_max_gyro_age_ms").as_double(), 0.0);
  params.driver.frame_id = node.get_parameter("frame_id").as_string();

  return params;
}
} // namespace OASIS::ROS
