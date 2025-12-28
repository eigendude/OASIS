/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050Node.h"

#include "Mpu6050NodeUtils.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{

// Default node name
constexpr const char* NODE_NAME = "mpu6050_imu_driver";

// ROS topics
constexpr const char* CONDUCTOR_STATE_TOPIC = "conductor_state";
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* IMU_DEBUG_TOPIC = "imu_debug";
constexpr const char* IMU_MAPPING_DEBUG_TOPIC = "imu_mapping_debug";
constexpr const char* IMU_STATUS_TOPIC = "imu_status";

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;
constexpr double DEFAULT_CONDUCTOR_STALE_SECONDS = 0.5;
constexpr double DEFAULT_EWMA_TAU = 2.0;

} // namespace

using namespace OASIS::ROS;
using namespace OASIS::ROS::Mpu6050NodeUtils;
using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node()
  : rclcpp::Node(NODE_NAME), m_motorIntent(DEFAULT_EWMA_TAU, DEFAULT_CONDUCTOR_STALE_SECONDS)
{
  OASIS::IMU::Mpu6050ImuProcessor::Config imuDefaults;
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("conductor_state_topic", std::string(CONDUCTOR_STATE_TOPIC));
  declare_parameter("imu_debug_topic", std::string(IMU_DEBUG_TOPIC));
  declare_parameter("imu_status_topic", std::string(IMU_STATUS_TOPIC));
  declare_parameter("imu_mapping_debug_topic", std::string(IMU_MAPPING_DEBUG_TOPIC));
  declare_parameter("stationary_gyro_thresh", imuDefaults.stationary_gyro_thresh);
  declare_parameter("stationary_accel_mag_thresh", imuDefaults.stationary_accel_mag_thresh);
  declare_parameter("stationary_hold_seconds", imuDefaults.stationary_hold_seconds);
  declare_parameter("mahony_kp", imuDefaults.mahony_kp);
  declare_parameter("mahony_ki", imuDefaults.mahony_ki);
  declare_parameter("mahony_integral_limit", imuDefaults.mahony_integral_limit);
  declare_parameter("bias_tau", imuDefaults.bias_tau);
  declare_parameter("bias_q", imuDefaults.bias_q);
  declare_parameter("bias_var_min", imuDefaults.bias_var_min);
  declare_parameter("bias_var_max", imuDefaults.bias_var_max);
  declare_parameter("ewma_tau", imuDefaults.ewma_tau);
  declare_parameter("relevel_rate", imuDefaults.relevel_rate);
  declare_parameter("accel_confidence_range", imuDefaults.accel_confidence_range);
  declare_parameter("gravity_lp_tau", imuDefaults.gravity_lp_tau);
  declare_parameter("accel_scale_noise", imuDefaults.accel_scale_noise);
  declare_parameter("gyro_scale_noise", imuDefaults.gyro_scale_noise);
  declare_parameter("temp_scale", imuDefaults.temp_scale);
  declare_parameter("conductor_stale_seconds", DEFAULT_CONDUCTOR_STALE_SECONDS);
  declare_parameter("dvdt_thresh", imuDefaults.dvdt_thresh);
  declare_parameter("alin_thresh", imuDefaults.alin_thresh);
  declare_parameter("forward_response_accel_thresh", imuDefaults.forward_response_accel_thresh);
  declare_parameter("forward_response_gyro_thresh", imuDefaults.forward_response_gyro_thresh);
  declare_parameter("forward_lock_seconds", imuDefaults.forward_lock_seconds);
  declare_parameter("forward_score_thresh", imuDefaults.forward_score_thresh);
  declare_parameter("forward_deadband", imuDefaults.forward_deadband);
  declare_parameter("forward_sign_consistency_samples", imuDefaults.forward_sign_consistency_samples);
  declare_parameter("forward_gyro_veto_z", imuDefaults.forward_gyro_veto_z);
  declare_parameter("forward_gyro_veto_xy", imuDefaults.forward_gyro_veto_xy);
  declare_parameter("yaw_inflate_factor", imuDefaults.yaw_inflate_factor);
  declare_parameter("dt_min", imuDefaults.dt_min);
  declare_parameter("dt_max", imuDefaults.dt_max);

  m_i2cDevice = get_parameter("i2c_device").as_string();
  m_conductorStateTopic = get_parameter("conductor_state_topic").as_string();
  m_imuDebugTopic = get_parameter("imu_debug_topic").as_string();
  m_imuStatusTopic = get_parameter("imu_status_topic").as_string();
  m_imuMappingDebugTopic = get_parameter("imu_mapping_debug_topic").as_string();
  const double stationaryGyroThresh = get_parameter("stationary_gyro_thresh").as_double();
  const double stationaryAccelMagThresh = get_parameter("stationary_accel_mag_thresh").as_double();
  const double stationaryHoldSeconds = get_parameter("stationary_hold_seconds").as_double();
  const double mahonyKp = get_parameter("mahony_kp").as_double();
  const double mahonyKi = get_parameter("mahony_ki").as_double();
  const double mahonyIntegralLimit = get_parameter("mahony_integral_limit").as_double();
  const double biasTau = get_parameter("bias_tau").as_double();
  const double biasQ = get_parameter("bias_q").as_double();
  const double biasVarMin = get_parameter("bias_var_min").as_double();
  const double biasVarMax = get_parameter("bias_var_max").as_double();
  const double ewmaTau = get_parameter("ewma_tau").as_double();
  const double relevelRate = get_parameter("relevel_rate").as_double();
  const double accelConfidenceRange = get_parameter("accel_confidence_range").as_double();
  const double gravityLpTau = get_parameter("gravity_lp_tau").as_double();
  const double accelScaleNoise = get_parameter("accel_scale_noise").as_double();
  const double gyroScaleNoise = get_parameter("gyro_scale_noise").as_double();
  const double tempScale = get_parameter("temp_scale").as_double();
  const double conductorStaleSeconds = get_parameter("conductor_stale_seconds").as_double();
  const double dvdtThresh = get_parameter("dvdt_thresh").as_double();
  const double alinThresh = get_parameter("alin_thresh").as_double();
  const double forwardResponseAccelThresh =
      get_parameter("forward_response_accel_thresh").as_double();
  const double forwardResponseGyroThresh = get_parameter("forward_response_gyro_thresh").as_double();
  const double forwardLockSeconds = get_parameter("forward_lock_seconds").as_double();
  const double forwardScoreThresh = get_parameter("forward_score_thresh").as_double();
  const double forwardDeadband = get_parameter("forward_deadband").as_double();
  const int forwardSignConsistencySamples =
      get_parameter("forward_sign_consistency_samples").as_int();
  const double forwardGyroVetoZ = get_parameter("forward_gyro_veto_z").as_double();
  const double forwardGyroVetoXY = get_parameter("forward_gyro_veto_xy").as_double();
  const double yawInflateFactor = get_parameter("yaw_inflate_factor").as_double();
  const double dtMin = get_parameter("dt_min").as_double();
  const double dtMax = get_parameter("dt_max").as_double();
  m_motorIntent = MotorIntent(ewmaTau, conductorStaleSeconds);
  OASIS::IMU::Mpu6050ImuProcessor::Config imuConfig;
  imuConfig.stationary_gyro_thresh = stationaryGyroThresh;
  imuConfig.stationary_accel_mag_thresh = stationaryAccelMagThresh;
  imuConfig.stationary_hold_seconds = stationaryHoldSeconds;
  imuConfig.mahony_kp = mahonyKp;
  imuConfig.mahony_ki = mahonyKi;
  imuConfig.mahony_integral_limit = mahonyIntegralLimit;
  imuConfig.bias_tau = biasTau;
  imuConfig.bias_q = biasQ;
  imuConfig.bias_var_min = biasVarMin;
  imuConfig.bias_var_max = biasVarMax;
  imuConfig.ewma_tau = ewmaTau;
  imuConfig.relevel_rate = relevelRate;
  imuConfig.accel_confidence_range = accelConfidenceRange;
  imuConfig.gravity_lp_tau = gravityLpTau;
  imuConfig.accel_scale_noise = accelScaleNoise;
  imuConfig.gyro_scale_noise = gyroScaleNoise;
  imuConfig.temp_scale = tempScale;
  imuConfig.dvdt_thresh = dvdtThresh;
  imuConfig.alin_thresh = alinThresh;
  imuConfig.forward_response_accel_thresh = forwardResponseAccelThresh;
  imuConfig.forward_response_gyro_thresh = forwardResponseGyroThresh;
  imuConfig.forward_lock_seconds = forwardLockSeconds;
  imuConfig.forward_score_thresh = forwardScoreThresh;
  imuConfig.forward_deadband = forwardDeadband;
  imuConfig.forward_sign_consistency_samples = forwardSignConsistencySamples;
  imuConfig.forward_gyro_veto_z = forwardGyroVetoZ;
  imuConfig.forward_gyro_veto_xy = forwardGyroVetoXY;
  imuConfig.yaw_inflate_factor = yawInflateFactor;
  imuConfig.dt_min = dtMin;
  imuConfig.dt_max = dtMax;
  m_imuProcessor = std::make_unique<OASIS::IMU::Mpu6050ImuProcessor>(imuConfig);

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
  const double accelScale = AccelScaleFromRange(accelRange);
  const double gyroScale = GyroScaleFromRange(gyroRange);
  if (m_imuProcessor)
  {
    m_imuProcessor->SetAccelScale(accelScale);
    m_imuProcessor->SetGyroScale(gyroScale);
  }

  RCLCPP_INFO(get_logger(), "MPU6050 full-scale ranges set (accel=%u, gyro=%u)",
              static_cast<unsigned>(accelRange), static_cast<unsigned>(gyroRange));

  // Initialize publishers
  m_publisher = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(10));
  m_debugPublisher =
      create_publisher<std_msgs::msg::Float64MultiArray>(m_imuDebugTopic, rclcpp::QoS(10));
  m_statusPublisher = create_publisher<std_msgs::msg::Bool>(m_imuStatusTopic, rclcpp::QoS(10));
  m_mappingDebugPublisher =
      create_publisher<std_msgs::msg::Float64MultiArray>(m_imuMappingDebugTopic, rclcpp::QoS(10));
  m_conductorStateSub = create_subscription<oasis_msgs::msg::ConductorState>(
      m_conductorStateTopic, rclcpp::QoS(10),
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
  const bool hasStamp = (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
  if (!hasStamp)
  {
    RCLCPP_ERROR(get_logger(), "ConductorState message missing timestamp");
    return;
  }

  const rclcpp::Time stamp{msg.header.stamp};
  const double dutyRaw = msg.duty_cycle;
  const double dutyClamped = std::clamp(dutyRaw, -1.0, 1.0);

  m_motorIntent.Update(stamp, dutyClamped);
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
  if (!m_imuProcessor)
    return;

  const MotorIntentOutput motorOutput = m_motorIntent.Get(now);
  OASIS::IMU::Mpu6050ImuProcessor::MotorIntentInput motorInput;
  motorInput.duty_raw = motorOutput.duty_raw;
  motorInput.duty_filt = motorOutput.duty_filt;
  motorInput.duty_dvdt = motorOutput.duty_dvdt;
  motorInput.fresh = motorOutput.fresh;
  motorInput.commanded = motorOutput.commanded;
  motorInput.sign = motorOutput.sign;

  OASIS::IMU::Mpu6050ImuProcessor::RawSample sample;
  sample.ax = ax;
  sample.ay = ay;
  sample.az = az;
  sample.gx = gx;
  sample.gy = gy;
  sample.gz = gz;
  sample.temp_raw = tempRaw;
  sample.timestamp = now.seconds();

  auto output = m_imuProcessor->Process(sample, motorInput);
  if (!output)
    return;

  if (output->mapping_jump_reason.has_value())
  {
    RCLCPP_INFO(get_logger(), "IMU mapping jump: %s", output->mapping_jump_reason->c_str());
  }
  for (const auto& message : output->info_messages)
  {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  auto imuMsg = sensor_msgs::msg::Imu();

  std_msgs::msg::Header& header = imuMsg.header;
  header.stamp = now;
  header.frame_id = FRAME_ID;

  geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  geometry_msgs::msg::Vector3& linearAcceleration = imuMsg.linear_acceleration;

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

  if (m_debugPublisher)
  {
    std_msgs::msg::Float64MultiArray debugMsg;
    debugMsg.data = output->debug_values;
    m_debugPublisher->publish(debugMsg);
  }
  if (m_statusPublisher && m_statusPublisher->get_subscription_count() > 0)
  {
    std_msgs::msg::Bool statusMsg;
    statusMsg.data = output->forward_solved;
    m_statusPublisher->publish(statusMsg);
  }
  if (m_mappingDebugPublisher && m_mappingDebugPublisher->get_subscription_count() > 0)
  {
    std_msgs::msg::Float64MultiArray mappingMsg;
    mappingMsg.data.reserve(9);
    for (const auto& row : output->active_mapping)
    {
      for (int value : row)
        mappingMsg.data.push_back(static_cast<double>(value));
    }
    m_mappingDebugPublisher->publish(mappingMsg);
  }

  m_publisher->publish(imuMsg);
}
