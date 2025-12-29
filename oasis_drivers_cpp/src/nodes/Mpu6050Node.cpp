/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050Node.h"

#include "imu/ImuMath.h"
#include "imu/ImuOrientationUtils.h"
#include "imu/Mpu6050ImuUtils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>

using namespace OASIS::IMU;
using namespace OASIS::ROS;
using namespace std::chrono_literals;

namespace
{

// Default node name
constexpr const char* NODE_NAME = "mpu6050_imu_driver";
constexpr double GRAVITY_MPS2 = 9.80665;

// ROS topics
constexpr const char* CONDUCTOR_STATE_TOPIC = "conductor_state";
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* IMU_FORWARD_TOPIC = "imu_forward";
constexpr const char* IMU_REVERSE_TOPIC = "imu_reverse";

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double kRollPitchVariance = 0.0076;
constexpr double kYawVarianceUnlocked = 1e6;
constexpr double kYawVarianceLocked = 0.02;
constexpr double kPi = 3.141592653589793;

const char* ToString(ForwardAxisLearner::State state)
{
  switch (state)
  {
    case ForwardAxisLearner::State::ARMED:
      return "ARMED";
    case ForwardAxisLearner::State::IN_BURST:
      return "IN_BURST";
    case ForwardAxisLearner::State::HAVE_FIRST:
      return "HAVE_FIRST";
    case ForwardAxisLearner::State::LOCKED:
      return "LOCKED";
  }
  return "UNKNOWN";
}

geometry_msgs::msg::Quaternion ToRosQuaternion(const Math::Quaternion& q)
{
  geometry_msgs::msg::Quaternion out;
  out.w = q.w;
  out.x = q.x;
  out.y = q.y;
  out.z = q.z;
  return out;
}

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

  const uint8_t who = m_mpu6050->getDeviceID();
  RCLCPP_INFO(get_logger(), "MPU6050 WHO_AM_I / device ID: 0x%02X", who);

  const uint8_t accelRange = m_mpu6050->getFullScaleAccelRange();
  const uint8_t gyroRange = m_mpu6050->getFullScaleGyroRange();

  // Initialize IMU state based on actual configuration
  const double accelScale = Mpu6050ImuUtils::AccelScaleFromRange(accelRange);
  const double gyroScale = Mpu6050ImuUtils::GyroScaleFromRange(gyroRange);

  RCLCPP_INFO(get_logger(),
              "MPU6050 ranges: accelRange=%u gyroRange=%u accelScale=%.9f m/s^2/LSB gyroScale=%.9f "
              "rad/s/LSB",
              static_cast<unsigned>(accelRange), static_cast<unsigned>(gyroRange), accelScale,
              gyroScale);

  // Also log raw register values (helps catch library enum vs register mismatch)
  const uint8_t regAccelCfg = m_mpu6050->getFullScaleAccelRange(); // same call, but log explicitly
  const uint8_t regGyroCfg = m_mpu6050->getFullScaleGyroRange();
  RCLCPP_INFO(get_logger(), "MPU6050 readback (enum): ACCEL_CFG=%u GYRO_CFG=%u",
              static_cast<unsigned>(regAccelCfg), static_cast<unsigned>(regGyroCfg));

  uint8_t accelConfig = 0;
  uint8_t gyroConfig = 0;
  uint8_t dlpfConfig = 0;
  uint8_t sampleRateDiv = 0;
  uint8_t powerMgmt = 0;

  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &accelConfig);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, &gyroConfig);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, &dlpfConfig);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, &sampleRateDiv);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, &powerMgmt);

  RCLCPP_INFO(get_logger(),
              "MPU6050 regs: ACCEL_CONFIG=0x%02X GYRO_CONFIG=0x%02X CONFIG(DLPF)=0x%02X "
              "SMPLRT_DIV=0x%02X PWR_MGMT_1=0x%02X",
              accelConfig, gyroConfig, dlpfConfig, sampleRateDiv, powerMgmt);

  m_imuProcessor.SetAccelScale(accelScale);
  m_imuProcessor.SetGyroScale(gyroScale);

  RCLCPP_INFO(get_logger(), "MPU6050 full-scale ranges set (accel=%u, gyro=%u)",
              static_cast<unsigned>(accelRange), static_cast<unsigned>(gyroRange));

  // Initialize publishers
  m_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS{1});
  m_imuForwardPublisher =
      create_publisher<sensor_msgs::msg::Imu>(IMU_FORWARD_TOPIC, rclcpp::QoS{1});
  m_imuReversePublisher =
      create_publisher<sensor_msgs::msg::Imu>(IMU_REVERSE_TOPIC, rclcpp::QoS{1});
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

  const rclcpp::Time now = get_clock()->now();
  double dt_seconds = 0.0;
  if (m_hasLastSampleTime)
  {
    dt_seconds = (now - m_lastSampleTime).seconds();
  }
  m_lastSampleTime = now;
  m_hasLastSampleTime = true;

  // MPU6050 datasheet formula: Temp(°C) = (TEMP_OUT / 340) + 36.53
  const int16_t tempRaw = m_mpu6050->getTemperature();
  const double temp_c = (static_cast<double>(tempRaw) / 340.0) + 36.53;

  const bool drdy = m_mpu6050->getIntDataReadyStatus();

  const auto processed = m_imuProcessor.ProcessRaw(ax, ay, az, gx, gy, gz, dt_seconds);

  if (processed.boot_accel_scale_applied)
  {
    RCLCPP_INFO(get_logger(),
                "Auto-trimmed accel sensitivity: lsb_per_g=%.3f new_accelScale=%.9f m/s^2/LSB",
                processed.boot_lsb_per_g, processed.boot_accel_scale);
  }

  const double accel_norm_g = std::sqrt(processed.accel_mps2[0] * processed.accel_mps2[0] +
                                        processed.accel_mps2[1] * processed.accel_mps2[1] +
                                        processed.accel_mps2[2] * processed.accel_mps2[2]) /
                              GRAVITY_MPS2;
  const double raw_accel_norm_lsb = std::sqrt(static_cast<double>(ax) * static_cast<double>(ax) +
                                              static_cast<double>(ay) * static_cast<double>(ay) +
                                              static_cast<double>(az) * static_cast<double>(az));
  const double accel_scale = m_imuProcessor.GetAccelScale();
  const double raw_accel_norm_mps2 = raw_accel_norm_lsb * accel_scale;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "IMU accel scale=%.9f m/s^2/LSB raw |a|=%.3f m/s^2 (%.1f LSB)", accel_scale,
                       raw_accel_norm_mps2, raw_accel_norm_lsb);

  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IMU accel=(%.3f, %.3f, %.3f) m/s^2 |g|=%.3f stationary=%s coverage=x(+:%s -:%s) "
      "y(+:%s -:%s) z(+:%s -:%s) bias=(%.4f, %.4f, %.4f) "
      "scale=(%.4f, %.4f, %.4f) temp=%.2fC data_ready=%s",
      processed.accel_mps2[0], processed.accel_mps2[1], processed.accel_mps2[2], accel_norm_g,
      processed.diag.stationary ? "true" : "false", processed.diag.pos_seen[0] ? "true" : "false",
      processed.diag.neg_seen[0] ? "true" : "false", processed.diag.pos_seen[1] ? "true" : "false",
      processed.diag.neg_seen[1] ? "true" : "false", processed.diag.pos_seen[2] ? "true" : "false",
      processed.diag.neg_seen[2] ? "true" : "false", processed.diag.bias_mps2[0],
      processed.diag.bias_mps2[1], processed.diag.bias_mps2[2], processed.diag.scale[0],
      processed.diag.scale[1], processed.diag.scale[2], temp_c, drdy ? "true" : "false");

  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "ForwardAxis state=%s yaw_rate=%.3f a_h=%.3f impulse=%.3f yaw_accum=%.3f t_burst=%.3f "
      "bursts=%zu dot=%.3f locked=%s",
      ToString(processed.forward_diag.state), processed.forward_diag.yaw_rate_rads,
      processed.forward_diag.a_h_mag_mps2, processed.forward_diag.impulse_mps,
      processed.forward_diag.yaw_accum_rad, processed.forward_diag.burst_time_s,
      processed.forward_diag.burst_count, processed.forward_diag.consistency_dot,
      processed.forward_diag.locked ? "true" : "false");

  const Math::Quaternion tilt_q =
      ImuOrientationUtils::TiltQuaternionFromUp(processed.u_hat, processed.u_hat_valid);
  const Math::Quaternion yaw_pi_q = Math::FromAxisAngle({0.0, 0.0, 1.0}, kPi);
  const Math::Quaternion reverse_q = Math::Multiply(yaw_pi_q, tilt_q);

  auto fillImuMsg =
      [&](sensor_msgs::msg::Imu& msg, const Math::Quaternion& orientation, bool yaw_defined)
  {
    std_msgs::msg::Header& header = msg.header;
    header.stamp = now;
    header.frame_id = FRAME_ID;

    geometry_msgs::msg::Vector3& angularVelocity = msg.angular_velocity;
    geometry_msgs::msg::Vector3& linearAcceleration = msg.linear_acceleration;

    linearAcceleration.x = processed.accel_mps2[0];
    linearAcceleration.y = processed.accel_mps2[1];
    linearAcceleration.z = processed.accel_mps2[2];

    angularVelocity.x = processed.gyro_rads[0];
    angularVelocity.y = processed.gyro_rads[1];
    angularVelocity.z = processed.gyro_rads[2];

    msg.orientation = ToRosQuaternion(orientation);

    msg.orientation_covariance.fill(0.0);
    msg.orientation_covariance[0] = kRollPitchVariance;
    msg.orientation_covariance[4] = kRollPitchVariance;
    msg.orientation_covariance[8] = yaw_defined ? kYawVarianceLocked : kYawVarianceUnlocked;
    msg.angular_velocity_covariance.fill(0.0);
    msg.linear_acceleration_covariance.fill(0.0);
  };

  sensor_msgs::msg::Imu imuMsg;
  sensor_msgs::msg::Imu imuForwardMsg;
  sensor_msgs::msg::Imu imuReverseMsg;

  fillImuMsg(imuMsg, tilt_q, false);
  fillImuMsg(imuForwardMsg, tilt_q, processed.f_hat_locked);
  fillImuMsg(imuReverseMsg, reverse_q, processed.f_hat_locked);

  m_imuPublisher->publish(imuMsg);
  m_imuForwardPublisher->publish(imuForwardMsg);
  m_imuReversePublisher->publish(imuReverseMsg);
}
