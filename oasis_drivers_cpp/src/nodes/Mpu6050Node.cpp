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

} // namespace

using namespace OASIS::ROS;
using namespace OASIS::ROS::Mpu6050NodeUtils;
using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("conductor_state_topic", std::string(CONDUCTOR_STATE_TOPIC));
  declare_parameter("imu_debug_topic", std::string(IMU_DEBUG_TOPIC));
  declare_parameter("imu_status_topic", std::string(IMU_STATUS_TOPIC));
  declare_parameter("imu_mapping_debug_topic", std::string(IMU_MAPPING_DEBUG_TOPIC));
  declare_parameter("stationary_gyro_thresh", m_stationaryGyroThresh);
  declare_parameter("stationary_accel_mag_thresh", m_stationaryAccelMagThresh);
  declare_parameter("stationary_hold_seconds", m_stationaryHoldSeconds);
  declare_parameter("mahony_kp", m_kpBase);
  declare_parameter("mahony_ki", m_kiBase);
  declare_parameter("mahony_integral_limit", m_mahonyIntegralLimit);
  declare_parameter("bias_tau", m_biasTau);
  declare_parameter("bias_q", m_biasQ);
  declare_parameter("bias_var_min", m_biasVarMin);
  declare_parameter("bias_var_max", m_biasVarMax);
  declare_parameter("ewma_tau", m_ewmaTau);
  declare_parameter("relevel_rate", m_relevelRate);
  declare_parameter("accel_confidence_range", m_accelConfidenceRange);
  declare_parameter("gravity_lp_tau", m_gravityLpTau);
  declare_parameter("accel_scale_noise", m_accelScaleNoise);
  declare_parameter("gyro_scale_noise", m_gyroScaleNoise);
  declare_parameter("temp_scale", m_tempScale);
  declare_parameter("conductor_stale_seconds", m_conductorStaleSeconds);
  declare_parameter("dvdt_thresh", m_dvdtThresh);
  declare_parameter("alin_thresh", m_alinThresh);
  declare_parameter("forward_response_accel_thresh", m_forwardResponseAccelThresh);
  declare_parameter("forward_response_gyro_thresh", m_forwardResponseGyroThresh);
  declare_parameter("forward_lock_seconds", m_forwardLockSeconds);
  declare_parameter("forward_score_thresh", m_forwardScoreThresh);
  declare_parameter("forward_deadband", m_forwardDeadband);
  declare_parameter("forward_sign_consistency_samples", m_forwardSignConsistencySamples);
  declare_parameter("forward_gyro_veto_z", m_forwardGyroVetoZ);
  declare_parameter("forward_gyro_veto_xy", m_forwardGyroVetoXY);
  declare_parameter("yaw_inflate_factor", m_yawInflateFactor);
  declare_parameter("dt_min", m_dtMin);
  declare_parameter("dt_max", m_dtMax);

  m_i2cDevice = get_parameter("i2c_device").as_string();
  m_conductorStateTopic = get_parameter("conductor_state_topic").as_string();
  m_imuDebugTopic = get_parameter("imu_debug_topic").as_string();
  m_imuStatusTopic = get_parameter("imu_status_topic").as_string();
  m_imuMappingDebugTopic = get_parameter("imu_mapping_debug_topic").as_string();
  m_stationaryGyroThresh = get_parameter("stationary_gyro_thresh").as_double();
  m_stationaryAccelMagThresh = get_parameter("stationary_accel_mag_thresh").as_double();
  m_stationaryHoldSeconds = get_parameter("stationary_hold_seconds").as_double();
  m_kpBase = get_parameter("mahony_kp").as_double();
  m_kiBase = get_parameter("mahony_ki").as_double();
  m_mahonyIntegralLimit = get_parameter("mahony_integral_limit").as_double();
  m_biasTau = get_parameter("bias_tau").as_double();
  m_biasQ = get_parameter("bias_q").as_double();
  m_biasVarMin = get_parameter("bias_var_min").as_double();
  m_biasVarMax = get_parameter("bias_var_max").as_double();
  m_ewmaTau = get_parameter("ewma_tau").as_double();
  m_relevelRate = get_parameter("relevel_rate").as_double();
  m_accelConfidenceRange = get_parameter("accel_confidence_range").as_double();
  m_gravityLpTau = get_parameter("gravity_lp_tau").as_double();
  m_accelScaleNoise = get_parameter("accel_scale_noise").as_double();
  m_gyroScaleNoise = get_parameter("gyro_scale_noise").as_double();
  m_tempScale = get_parameter("temp_scale").as_double();
  m_conductorStaleSeconds = get_parameter("conductor_stale_seconds").as_double();
  m_dvdtThresh = get_parameter("dvdt_thresh").as_double();
  m_alinThresh = get_parameter("alin_thresh").as_double();
  m_forwardResponseAccelThresh = get_parameter("forward_response_accel_thresh").as_double();
  m_forwardResponseGyroThresh = get_parameter("forward_response_gyro_thresh").as_double();
  m_forwardLockSeconds = get_parameter("forward_lock_seconds").as_double();
  m_forwardScoreThresh = get_parameter("forward_score_thresh").as_double();
  m_forwardDeadband = get_parameter("forward_deadband").as_double();
  m_forwardSignConsistencySamples = get_parameter("forward_sign_consistency_samples").as_int();
  m_forwardGyroVetoZ = get_parameter("forward_gyro_veto_z").as_double();
  m_forwardGyroVetoXY = get_parameter("forward_gyro_veto_xy").as_double();
  m_yawInflateFactor = get_parameter("yaw_inflate_factor").as_double();
  m_dtMin = get_parameter("dt_min").as_double();
  m_dtMax = get_parameter("dt_max").as_double();
  m_motorIntent = MotorIntent(m_ewmaTau, m_conductorStaleSeconds);
  OASIS::IMU::Mpu6050ImuProcessor::Config imuConfig;
  imuConfig.stationary_gyro_thresh = m_stationaryGyroThresh;
  imuConfig.stationary_accel_mag_thresh = m_stationaryAccelMagThresh;
  imuConfig.stationary_hold_seconds = m_stationaryHoldSeconds;
  imuConfig.mahony_kp = m_kpBase;
  imuConfig.mahony_ki = m_kiBase;
  imuConfig.mahony_integral_limit = m_mahonyIntegralLimit;
  imuConfig.bias_tau = m_biasTau;
  imuConfig.bias_q = m_biasQ;
  imuConfig.bias_var_min = m_biasVarMin;
  imuConfig.bias_var_max = m_biasVarMax;
  imuConfig.ewma_tau = m_ewmaTau;
  imuConfig.relevel_rate = m_relevelRate;
  imuConfig.accel_confidence_range = m_accelConfidenceRange;
  imuConfig.gravity_lp_tau = m_gravityLpTau;
  imuConfig.accel_scale_noise = m_accelScaleNoise;
  imuConfig.gyro_scale_noise = m_gyroScaleNoise;
  imuConfig.temp_scale = m_tempScale;
  imuConfig.dvdt_thresh = m_dvdtThresh;
  imuConfig.alin_thresh = m_alinThresh;
  imuConfig.forward_response_accel_thresh = m_forwardResponseAccelThresh;
  imuConfig.forward_response_gyro_thresh = m_forwardResponseGyroThresh;
  imuConfig.forward_lock_seconds = m_forwardLockSeconds;
  imuConfig.forward_score_thresh = m_forwardScoreThresh;
  imuConfig.forward_deadband = m_forwardDeadband;
  imuConfig.forward_sign_consistency_samples = m_forwardSignConsistencySamples;
  imuConfig.forward_gyro_veto_z = m_forwardGyroVetoZ;
  imuConfig.forward_gyro_veto_xy = m_forwardGyroVetoXY;
  imuConfig.yaw_inflate_factor = m_yawInflateFactor;
  imuConfig.dt_min = m_dtMin;
  imuConfig.dt_max = m_dtMax;
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
