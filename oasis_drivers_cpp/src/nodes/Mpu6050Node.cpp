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
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>

using namespace OASIS::ROS;
using namespace std::chrono_literals;

namespace
{

// Default node name
constexpr const char* NODE_NAME = "mpu6050_imu_driver";

// ROS topics
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* IMU_RAW_TOPIC = "imu_raw";
constexpr const char* IMU_TEMPERATURE_TOPIC = "imu_temperature";

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double DEFAULT_GRAVITY = 9.80665; // m/s^2
constexpr const char* DEFAULT_IMU_CALIBRATION_BASE = "imu_mpu6050_calibration";

// Filesystem parameters
constexpr const char* ROS_PROFILE_DIR_NAME = ".ros";
constexpr const char* IMU_INFO_DIR_NAME = "imu_info";
} // namespace

Mpu6050Node::Mpu6050Node() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("gravity", DEFAULT_GRAVITY);
  declare_parameter("system_id", std::string(""));
  declare_parameter("imu_calibration_base", std::string(DEFAULT_IMU_CALIBRATION_BASE));

  m_i2cDevice = get_parameter("i2c_device").as_string();

  const double publishRateHz = get_parameter("publish_rate_hz").as_double();
  const double clampedRate = std::max(publishRateHz, 0.1);
  m_publishPeriod = std::chrono::duration<double>(1.0 / clampedRate);

  m_gravity = get_parameter("gravity").as_double();

  m_systemId = get_parameter("system_id").as_string();
  m_imuCalibrationBase = get_parameter("imu_calibration_base").as_string();

  if (m_systemId.empty())
  {
    RCLCPP_ERROR(get_logger(), "system_id parameter is empty");
    throw std::runtime_error("Missing system_id parameter");
  }

  // Get IMU info directory
  const char* home = std::getenv("HOME");
  const std::filesystem::path homePath =
      home ? std::filesystem::path(home) : std::filesystem::path(".");
  const std::filesystem::path imuInfoDirectory =
      homePath / ROS_PROFILE_DIR_NAME / IMU_INFO_DIR_NAME;

  // Create IMU info directory
  std::error_code imuInfoError;
  std::filesystem::create_directories(imuInfoDirectory, imuInfoError);
  if (imuInfoError)
  {
    RCLCPP_ERROR(get_logger(), "Failed to create IMU info directory at %s: %s",
                 imuInfoDirectory.c_str(), imuInfoError.message().c_str());
    throw std::runtime_error("Failed to create IMU info directory");
  }

  m_calibrationCachePath = imuInfoDirectory / (m_imuCalibrationBase + "_" + m_systemId + ".yaml");

  const bool calibrationFileExists = std::filesystem::exists(m_calibrationCachePath);

  RCLCPP_INFO(get_logger(), "IMU calibration file %s: %s",
              calibrationFileExists ? "found" : "NOT found", m_calibrationCachePath.c_str());
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

  // Query WHO_AM_I
  const uint8_t whoAmI = m_mpu6050->getDeviceID();

  RCLCPP_INFO(get_logger(), "Connected to MPU6050 on %s with device ID: 0x%02X",
              m_i2cDevice.c_str(), whoAmI);

  // Ensure full-scale ranges match our scaling assumptions
  m_mpu6050->setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  m_mpu6050->setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  const uint8_t accelRange = m_mpu6050->getFullScaleAccelRange();
  const uint8_t gyroRange = m_mpu6050->getFullScaleGyroRange();

  // Initialize IMU state based on actual configuration
  const double accelScale = IMU::Mpu6050ImuUtils::AccelScaleFromRange(accelRange, m_gravity);
  const double gyroScale = IMU::Mpu6050ImuUtils::GyroScaleFromRange(gyroRange);

  m_imuProcessor.SetGravity(m_gravity);
  m_imuProcessor.SetAccelScale(accelScale);
  m_imuProcessor.SetGyroScale(gyroScale);
  m_imuProcessor.Reset();

  RCLCPP_INFO(get_logger(), "MPU6050 full-scale ranges set (accel=%u, gyro=%u)",
              static_cast<unsigned>(accelRange), static_cast<unsigned>(gyroRange));

  m_imuProcessor.ConfigureCalibration(m_calibrationCachePath, FRAME_ID);
  const bool loadedCalibration = m_imuProcessor.LoadCachedCalibration();
  m_calibrationMode = !loadedCalibration;
  m_imuProcessor.SetCalibrationMode(m_calibrationMode);

  if (loadedCalibration)
  {
    RCLCPP_INFO(get_logger(), "Loaded accelerometer calibration cache: %s",
                m_calibrationCachePath.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Calibration cache missing, running online calibration: %s",
                m_calibrationCachePath.c_str());
  }

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
  m_imuProcessor.Reset();
  m_lastSampleTime.reset();
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
    // Log error and return
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MPU6050 data not ready");
    return;
  }

  // Capture motion
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  m_mpu6050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Create timestamp
  const rclcpp::Time now = get_clock()->now();
  const double dt_s = m_lastSampleTime ? (now - *m_lastSampleTime).seconds() : 0.0;
  const double timestamp_s = now.seconds();
  m_lastSampleTime = now;

  // Capture temperature before calibration logic so the sample participates
  // in the calibration statistics.
  const int16_t tempRaw = m_mpu6050->getTemperature();
  const auto tempSample = m_imuTemperature.ProcessRaw(tempRaw, dt_s);

  // Process motion
  const auto processed =
      m_imuProcessor.ProcessRaw(ax, ay, az, gx, gy, gz, dt_s, tempSample.temperatureC, timestamp_s);

  if (m_calibrationMode && processed.calibration_status.wrote_cache)
  {
    RCLCPP_INFO(get_logger(),
                "Calibration cached at %s; restart the node to use updated parameters",
                m_calibrationCachePath.c_str());
  }

  if (m_calibrationMode)
  {
    static double last_log_s = 0.0;
    if (timestamp_s - last_log_s >= 1.0)
    {
      last_log_s = timestamp_s;

      const auto& st = processed.calibration_status;
      const auto& raw = processed.imu_raw;
      const double accel_norm =
          std::sqrt(raw.accel_mps2[0] * raw.accel_mps2[0] + raw.accel_mps2[1] * raw.accel_mps2[1] +
                    raw.accel_mps2[2] * raw.accel_mps2[2]);

      RCLCPP_INFO(get_logger(),
                  "Cal: stationary=%d clusters=%zu samples=%zu axis_cov=%d solved=%d "
                  "wrote=%d |a|=%.3f",
                  st.stationary, st.num_clusters, st.num_pose_samples, st.has_axis_coverage,
                  st.has_solution, st.wrote_cache, accel_norm);
    }
  }

  // Create header for published messages
  std_msgs::msg::Header headerMsg;
  headerMsg.stamp = now;
  headerMsg.frame_id = FRAME_ID;

  // Publish motion
  sensor_msgs::msg::Imu imuMsg;
  sensor_msgs::msg::Imu imuRawMsg;

  auto fillImuMsg =
      [&](sensor_msgs::msg::Imu& imuMsg, const IMU::Mpu6050ImuProcessor::ImuSample& sample)
  {
    // Header
    imuMsg.header = headerMsg;

    // Linear acceleration
    imuMsg.linear_acceleration.x = sample.accel_mps2[0];
    imuMsg.linear_acceleration.y = sample.accel_mps2[1];
    imuMsg.linear_acceleration.z = sample.accel_mps2[2];

    // Angular velocity
    imuMsg.angular_velocity.x = sample.gyro_rads[0];
    imuMsg.angular_velocity.y = sample.gyro_rads[1];
    imuMsg.angular_velocity.z = sample.gyro_rads[2];

    // Orientation not provided, set identity
    imuMsg.orientation.x = 0.0;
    imuMsg.orientation.y = 0.0;
    imuMsg.orientation.z = 0.0;
    imuMsg.orientation.w = 1.0;

    // Covariances: initialize all to zero, then set diagonals
    imuMsg.orientation_covariance.fill(0.0);
    imuMsg.linear_acceleration_covariance.fill(0.0);
    imuMsg.angular_velocity_covariance.fill(0.0);

    // Orientation not estimated (set [0,0] to -1 per REP-145)
    imuMsg.orientation_covariance[0] = -1.0;

    // Linear acceleration covariance (diagonal)
    imuMsg.linear_acceleration_covariance[0] = sample.accel_var_mps2_2[0];
    imuMsg.linear_acceleration_covariance[4] = sample.accel_var_mps2_2[1];
    imuMsg.linear_acceleration_covariance[8] = sample.accel_var_mps2_2[2];

    // Angular velocity covariance (diagonal)
    imuMsg.angular_velocity_covariance[0] = sample.gyro_var_rads2_2[0];
    imuMsg.angular_velocity_covariance[4] = sample.gyro_var_rads2_2[1];
    imuMsg.angular_velocity_covariance[8] = sample.gyro_var_rads2_2[2];
  };

  // imu_raw: direct sensor measurements with measurement-noise covariances.
  fillImuMsg(imuRawMsg, processed.imu_raw);

  // imu: calibrated stream for ORB-SLAM3. Acceleration retains gravity
  // because ORB-SLAM3 expects specific force.
  fillImuMsg(imuMsg, processed.imu);

  m_imuPublisher->publish(imuMsg);
  m_imuRawPublisher->publish(imuRawMsg);

  // Publish temperature
  sensor_msgs::msg::Temperature temperatureMsg;

  temperatureMsg.header = headerMsg;
  temperatureMsg.temperature = tempSample.temperatureC;
  temperatureMsg.variance = tempSample.varianceC2;

  m_imuTemperaturePublisher->publish(temperatureMsg);

  // Log data
  const double raw_accel_norm_lsb = std::sqrt(static_cast<double>(ax) * static_cast<double>(ax) +
                                              static_cast<double>(ay) * static_cast<double>(ay) +
                                              static_cast<double>(az) * static_cast<double>(az));
  const double a_raw_mps2 = raw_accel_norm_lsb * m_imuProcessor.GetAccelScale();

  const auto& a = processed.imu.accel_mps2;
  const double a_cal_mps2 = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "IMU: |a_raw|=%.3f |a_cal|=%.3f m/s^2, temp=%.2fC", a_raw_mps2, a_cal_mps2,
                       tempSample.temperatureC);
}
