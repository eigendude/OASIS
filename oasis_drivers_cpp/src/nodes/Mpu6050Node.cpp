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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>

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

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double kRollPitchVariance = 0.0076;
constexpr double kYawVariance = 1e6;
constexpr double kPi = 3.141592653589793;
constexpr double kTwoPi = 2.0 * kPi;
constexpr bool kLogStationaryTransitions = false;

unsigned AccelRangeGFromAfsSel(uint8_t afs_sel)
{
  switch (afs_sel & 0x03)
  {
    case 0:
      return 2;
    case 1:
      return 4;
    case 2:
      return 8;
    case 3:
      return 16;
    default:
      return 0;
  }
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

  uint8_t accelConfig = 0;
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &accelConfig);
  const uint8_t accelConfigForced = static_cast<uint8_t>(accelConfig & 0xE0);
  I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, accelConfigForced);

  uint8_t accelConfigForcedReadback = 0;
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &accelConfigForcedReadback);
  const uint8_t accelForcedAfsSel = static_cast<uint8_t>((accelConfigForcedReadback >> 3) & 0x03);
  const uint8_t accelForcedHpf = static_cast<uint8_t>(accelConfigForcedReadback & 0x07);
  const unsigned accelForcedRangeG = AccelRangeGFromAfsSel(accelForcedAfsSel);

  RCLCPP_INFO(
      get_logger(),
      "MPU6050 forced ACCEL_CONFIG=0x%02X AFS_SEL=%u (±%ug) ACCEL_HPF=%u (self-test=0x%02X)",
      accelConfigForcedReadback, static_cast<unsigned>(accelForcedAfsSel), accelForcedRangeG,
      static_cast<unsigned>(accelForcedHpf), static_cast<unsigned>(accelConfigForcedReadback >> 5));
  if (accelForcedAfsSel != 0 || accelForcedHpf != 0)
  {
    RCLCPP_WARN(get_logger(), "MPU6050 ACCEL_CONFIG readback AFS_SEL=%u ACCEL_HPF=%u (expected 0)",
                static_cast<unsigned>(accelForcedAfsSel),
                static_cast<unsigned>(accelForcedHpf));
  }

  const uint8_t who = m_mpu6050->getDeviceID();
  RCLCPP_INFO(get_logger(), "MPU6050 WHO_AM_I / device ID: 0x%02X", who);

  const uint8_t accelRange = m_mpu6050->getFullScaleAccelRange();
  const uint8_t gyroRange = m_mpu6050->getFullScaleGyroRange();

  // Initialize IMU state based on actual configuration
  const double accelScale = Mpu6050ImuUtils::AccelScaleFromRange(accelRange);
  const double gyroScale = Mpu6050ImuUtils::GyroScaleFromRange(gyroRange);
  const double accel_lsb_per_g = GRAVITY_MPS2 / accelScale;

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

  uint8_t accelConfigReadback = 0;
  uint8_t gyroConfig = 0;
  uint8_t dlpfConfig = 0;
  uint8_t sampleRateDiv = 0;
  uint8_t powerMgmt = 0;

  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &accelConfigReadback);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, &gyroConfig);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, &dlpfConfig);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, &sampleRateDiv);
  I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, &powerMgmt);

  const uint8_t accelAfsSel = static_cast<uint8_t>((accelConfigReadback >> 3) & 0x03);
  const uint8_t accelHpf = static_cast<uint8_t>(accelConfigReadback & 0x07);
  const unsigned accelRangeG = AccelRangeGFromAfsSel(accelAfsSel);

  RCLCPP_INFO(get_logger(),
              "MPU6050 ACCEL_CONFIG=0x%02X AFS_SEL=%u (±%ug) ACCEL_HPF=%u accel_lsb_per_g=%.1f",
              accelConfigReadback, static_cast<unsigned>(accelAfsSel), accelRangeG,
              static_cast<unsigned>(accelHpf), accel_lsb_per_g);
  if (accelHpf != 0)
  {
    RCLCPP_WARN(get_logger(), "MPU6050 ACCEL_HPF readback=%u (expected 0)",
                static_cast<unsigned>(accelHpf));
  }

  RCLCPP_INFO(get_logger(),
              "MPU6050 regs: ACCEL_CONFIG=0x%02X GYRO_CONFIG=0x%02X CONFIG(DLPF)=0x%02X "
              "SMPLRT_DIV=0x%02X PWR_MGMT_1=0x%02X",
              accelConfigReadback, gyroConfig, dlpfConfig, sampleRateDiv, powerMgmt);

  for (int i = 0; i < 5; ++i)
  {
    uint8_t startupAccelConfig = 0;
    const int8_t configBytesRead =
        I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &startupAccelConfig);
    if (configBytesRead <= 0)
    {
      RCLCPP_WARN(get_logger(), "MPU6050 startup ACCEL_CONFIG read failed");
      std::this_thread::sleep_for(1s);
      continue;
    }

    uint8_t rawBuffer[6] = {};
    const int8_t accelBytesRead = I2Cdev::readBytes(
        MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 6, rawBuffer);
    if (accelBytesRead != 6)
    {
      RCLCPP_WARN(get_logger(), "MPU6050 startup accel read failed (bytes=%d)",
                  static_cast<int>(accelBytesRead));
      std::this_thread::sleep_for(1s);
      continue;
    }

    const int16_t ax = static_cast<int16_t>((rawBuffer[0] << 8) | rawBuffer[1]);
    const int16_t ay = static_cast<int16_t>((rawBuffer[2] << 8) | rawBuffer[3]);
    const int16_t az = static_cast<int16_t>((rawBuffer[4] << 8) | rawBuffer[5]);
    const double accelNormLsb =
        std::sqrt(static_cast<double>(ax) * ax + static_cast<double>(ay) * ay +
                  static_cast<double>(az) * az);

    const uint8_t accelAfsSelStartup = static_cast<uint8_t>((startupAccelConfig >> 3) & 0x03);
    const uint8_t accelHpfStartup = static_cast<uint8_t>(startupAccelConfig & 0x07);
    const unsigned accelRangeStartupG = AccelRangeGFromAfsSel(accelAfsSelStartup);

    RCLCPP_INFO(
        get_logger(),
        "MPU6050 startup ACCEL_CONFIG=0x%02X AFS_SEL=%u (±%ug) ACCEL_HPF=%u ax=%d ay=%d az=%d "
        "|a|=%.1f LSB",
        startupAccelConfig, static_cast<unsigned>(accelAfsSelStartup), accelRangeStartupG,
        static_cast<unsigned>(accelHpfStartup), static_cast<int>(ax), static_cast<int>(ay),
        static_cast<int>(az), accelNormLsb);

    if (accelAfsSelStartup != 0 || accelHpfStartup != 0)
    {
      RCLCPP_WARN(get_logger(),
                  "MPU6050 startup ACCEL_CONFIG AFS_SEL=%u ACCEL_HPF=%u (expected 0)",
                  static_cast<unsigned>(accelAfsSelStartup),
                  static_cast<unsigned>(accelHpfStartup));
    }

    std::this_thread::sleep_for(1s);
  }

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
  const double tempC = (static_cast<double>(tempRaw) / 340.0) + 36.53;

  const bool dataReady = m_mpu6050->getIntDataReadyStatus();

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
      "IMU accel=(%.3f, %.3f, %.3f) m/s^2 |g|=%.3f stationary=%s stationary_confirmed=%s "
      "phase2=%s dwell=%.2f/%.2f coverage=x(+:%s -:%s) y(+:%s -:%s) z(+:%s -:%s) "
      "bias=(%.4f, %.4f, %.4f) scale=(%.4f, %.4f, %.4f) temp=%.2fC data_ready=%s",
      processed.accel_mps2[0], processed.accel_mps2[1], processed.accel_mps2[2], accel_norm_g,
      processed.diag.stationary ? "true" : "false",
      processed.diag.stationary_confirmed ? "true" : "false",
      processed.diag.stationary_phase2 ? "true" : "false", processed.diag.stationary_dwell_seconds,
      processed.diag.stationary_dwell_target_seconds, processed.diag.pos_seen[0] ? "true" : "false",
      processed.diag.neg_seen[0] ? "true" : "false", processed.diag.pos_seen[1] ? "true" : "false",
      processed.diag.neg_seen[1] ? "true" : "false", processed.diag.pos_seen[2] ? "true" : "false",
      processed.diag.neg_seen[2] ? "true" : "false", processed.diag.bias_mps2[0],
      processed.diag.bias_mps2[1], processed.diag.bias_mps2[2], processed.diag.scale[0],
      processed.diag.scale[1], processed.diag.scale[2], tempC, dataReady ? "true" : "false");

  if (kLogStationaryTransitions)
  {
    if (processed.diag.stationary_strict != m_prevStationaryStrict)
    {
      RCLCPP_INFO(get_logger(), "IMU strict stationary %s",
                  processed.diag.stationary_strict ? "entered" : "exited");
    }
    if (processed.diag.stationary_confirmed && !m_prevStationaryConfirmed)
    {
      RCLCPP_INFO(get_logger(), "IMU stationary dwell confirmed (%.2fs)",
                  processed.diag.stationary_dwell_seconds);
    }
  }
  m_prevStationaryStrict = processed.diag.stationary_strict;
  m_prevStationaryConfirmed = processed.diag.stationary_confirmed;

  const Math::Quaternion tilt_q =
      ImuOrientationUtils::TiltQuaternionFromUp(processed.u_hat, processed.u_hat_valid);
  if (dt_seconds > 0.0)
  {
    m_yawRad += processed.gyro_rads[2] * dt_seconds;
    m_yawRad = std::remainder(m_yawRad, kTwoPi);
  }
  const Math::Quaternion yaw_q = Math::FromAxisAngle({0.0, 0.0, 1.0}, m_yawRad);
  const Math::Quaternion orientation_q = Math::Multiply(yaw_q, tilt_q);

  auto fillImuMsg = [&](sensor_msgs::msg::Imu& msg, const Math::Quaternion& orientation)
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
    msg.orientation_covariance[8] = kYawVariance;
    msg.angular_velocity_covariance.fill(0.0);
    msg.linear_acceleration_covariance.fill(0.0);
  };

  sensor_msgs::msg::Imu imuMsg;

  fillImuMsg(imuMsg, orientation_q);

  m_imuPublisher->publish(imuMsg);
}
