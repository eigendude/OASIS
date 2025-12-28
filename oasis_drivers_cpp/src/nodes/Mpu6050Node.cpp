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
  m_accelScale = AccelScaleFromRange(accelRange);
  m_gyroScale = GyroScaleFromRange(gyroRange);

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

  if (!m_dutyFiltInit)
  {
    m_dutyFilt = dutyRaw;
    m_dutyFiltDvdt = 0.0;
    m_dutyFiltInit = true;
    m_lastConductorStamp = stamp;
    m_duty = dutyRaw;
    return;
  }

  const double dt = (stamp - m_lastConductorStamp).seconds();
  if (dt <= 0.0 || dt > 1.0) // guard out-of-order / stale bursts
  {
    m_lastConductorStamp = stamp;
    m_duty = dutyRaw;
    return;
  }

  // TODO

  m_lastConductorStamp = stamp;
  m_duty = dutyRaw;
}

void Mpu6050Node::OnMappingJump(const char* reason)
{
  m_yawVar = std::max(m_yawVar, 1.5);
  RCLCPP_INFO(get_logger(), "IMU mapping jump: %s", reason);
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
  m_hasTemperature = true;
  m_lastTemperature = static_cast<double>(tempRaw) / 340.0 + 36.53;

  auto imuMsg = sensor_msgs::msg::Imu();

  std_msgs::msg::Header& header = imuMsg.header;
  header.stamp = get_clock()->now();
  header.frame_id = FRAME_ID;

  geometry_msgs::msg::Vector3& angularVelocity = imuMsg.angular_velocity;
  geometry_msgs::msg::Vector3& linearAcceleration = imuMsg.linear_acceleration;

  const Vec3 accelRaw = {static_cast<double>(ax) * m_accelScale,
                         static_cast<double>(ay) * m_accelScale,
                         static_cast<double>(az) * m_accelScale};
  const Vec3 gyroRaw = {static_cast<double>(gx) * m_gyroScale,
                        static_cast<double>(gy) * m_gyroScale,
                        static_cast<double>(gz) * m_gyroScale};

  const rclcpp::Time now = header.stamp;
  if (m_lastStamp.nanoseconds() == 0)
  {
    m_lastStamp = now;
    return;
  }
  const double dt = (now - m_lastStamp).seconds();
  m_lastStamp = now;
  if (dt < m_dtMin || dt > m_dtMax)
    return;

  static const std::vector<Mapping> mappings = GenerateMappings();
  static Vec3 accelMean{0.0, 0.0, 0.0};
  static bool accelMeanInit = false;

  const Vec3 gyroMapped = ApplyMapping(m_activeMapping, gyroRaw);
  const Vec3 gyroUnbiasedPreSolve = Sub(gyroMapped, m_gyroBias);
  const double gyroMagPreSolve = Norm(gyroUnbiasedPreSolve);

  if (!m_zUpSolved)
  {
    const double accelMag = Norm(accelRaw);
    const bool accelOk = std::abs(accelMag - GRAVITY) < m_stationaryAccelMagThresh;
    if (gyroMagPreSolve < m_stationaryGyroThresh && accelOk)
    {
      const double alpha = EwmaAlpha(dt, m_ewmaTau);
      if (!accelMeanInit)
      {
        accelMean = accelRaw;
        accelMeanInit = true;
      }
      else
      {
        accelMean = Add(accelMean, Scale(Sub(accelRaw, accelMean), alpha));
      }
      m_stationaryGoodDuration += dt;
    }
    else
    {
      m_stationaryGoodDuration = 0.0;
    }

    if (accelMeanInit && m_stationaryGoodDuration >= m_stationaryHoldSeconds)
    {
      double bestScore = -1e9;
      Mapping best = m_zUpMapping;
      for (const auto& mapping : mappings)
      {
        const Vec3 mapped = ApplyMapping(mapping, accelMean);
        // Penalize Z magnitude too far from +g to avoid selecting a tilted axis.
        const double score = mapped[2] - 0.5 * (std::abs(mapped[0]) + std::abs(mapped[1])) -
                             Z_UP_SCORE_GRAVITY_PENALTY * std::abs(mapped[2] - GRAVITY);
        if (score > bestScore)
        {
          bestScore = score;
          best = mapping;
        }
      }
      const Vec3 mappedBest = ApplyMapping(best, accelMean);
      const bool zUpOk = mappedBest[2] > 0.8 * GRAVITY && std::abs(mappedBest[0]) < 0.3 * GRAVITY &&
                         std::abs(mappedBest[1]) < 0.3 * GRAVITY;
      if (zUpOk)
      {
        const Mapping oldMapping = m_activeMapping;
        m_zUpMapping = best;
        m_activeMapping = best;
        m_zUpSolved = true;
        m_stationaryGoodDuration = 0.0;
        if (m_hasTemperature)
          m_temperatureAtCal = m_lastTemperature;
        const Mapping delta = MultiplyMapping(m_activeMapping, TransposeMapping(oldMapping));
        const Quaternion qDelta = QuatFromRotationMatrix(delta);
        Quaternion q = {m_orientationQuat[0], m_orientationQuat[1], m_orientationQuat[2],
                        m_orientationQuat[3]};
        q = MultiplyQuat(q, ConjugateQuat(qDelta));
        q = NormalizeQuat(q);
        m_orientationQuat = {q[0], q[1], q[2], q[3]};
        OnMappingJump("Z-up solve");
        RCLCPP_INFO(get_logger(), "IMU mapping leveled to +Z");
      }
    }
  }

  Vec3 accel = ApplyMapping(m_activeMapping, accelRaw);
  Vec3 gyro = ApplyMapping(m_activeMapping, gyroRaw);
  const Vec3 gyroUnbiased = Sub(gyro, m_gyroBias);

  const double accelMag = Norm(accel);
  const double gyroUnbiasedNorm = Norm(gyroUnbiased);
  const bool accelOk = std::abs(accelMag - GRAVITY) < m_stationaryAccelMagThresh;
  const bool gyroOk = gyroUnbiasedNorm < m_stationaryGyroThresh;
  const bool stationaryNow = accelOk && gyroOk;
  if (stationaryNow)
  {
    m_stationaryDuration += dt;
    if (!m_isStationary && m_stationaryDuration >= m_stationaryHoldSeconds)
    {
      m_isStationary = true;
      RCLCPP_INFO(get_logger(), "IMU stationary detected");
    }
  }
  else
  {
    if (m_isStationary)
      RCLCPP_INFO(get_logger(), "IMU moving detected");
    m_isStationary = false;
    m_stationaryDuration = 0.0;
  }

  const double accelConfidence =
      std::clamp(1.0 - std::abs(accelMag - GRAVITY) / m_accelConfidenceRange, 0.0, 1.0);

  Quaternion q = {m_orientationQuat[0], m_orientationQuat[1], m_orientationQuat[2],
                  m_orientationQuat[3]};
  const Vec3 gravityBody = RotateVec(ConjugateQuat(q), {0.0, 0.0, GRAVITY});
  const Vec3 accelNorm = accelMag > 1e-6 ? Scale(accel, 1.0 / accelMag) : Vec3{0.0, 0.0, 0.0};
  const Vec3 gravityDir = Normalize(gravityBody);
  const Vec3 error = Cross(gravityDir, accelNorm);

  const double kpEff = m_kpBase * accelConfidence;
  const double kiEff = m_kiBase * accelConfidence;

  m_mahonyIntegral = Add(m_mahonyIntegral, Scale(error, kiEff * dt));
  // Clamp integrator to limit windup during sustained linear acceleration.
  for (size_t i = 0; i < 3; ++i)
    m_mahonyIntegral[i] =
        std::clamp(m_mahonyIntegral[i], -m_mahonyIntegralLimit, m_mahonyIntegralLimit);
  Vec3 gyroCorrected = gyroUnbiased;
  gyroCorrected = Add(gyroCorrected, Add(Scale(error, kpEff), m_mahonyIntegral));

  const double omegaNorm = Norm(gyroCorrected);
  if (omegaNorm > 1e-9)
  {
    const Quaternion dq = QuatFromAxisAngle(Scale(gyroCorrected, 1.0 / omegaNorm), omegaNorm * dt);
    q = MultiplyQuat(q, dq);
  }
  q = NormalizeQuat(q);

  if (m_isStationary && accelConfidence > 0.7)
  {
    const Vec3 euler = EulerFromQuat(q);
    const double roll = std::atan2(accel[1], accel[2]);
    const double pitch =
        std::atan2(-accel[0], std::sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
    const Quaternion target = QuatFromEuler(roll, pitch, euler[2]);
    q = Slerp(q, target, std::clamp(m_relevelRate * dt, 0.0, 1.0));
  }

  m_orientationQuat = {q[0], q[1], q[2], q[3]};

  const Vec3 gravityBodyUpdated = RotateVec(ConjugateQuat(q), {0.0, 0.0, GRAVITY});

  if (m_isStationary && accelConfidence > 0.7)
  {
    const double alphaBias = EwmaAlpha(dt, m_biasTau);
    m_gyroBias = Add(m_gyroBias, Scale(Sub(gyro, m_gyroBias), alphaBias));
    for (size_t i = 0; i < 3; ++i)
    {
      m_gyroBiasVar[i] =
          std::clamp(m_gyroBiasVar[i] * (1.0 - alphaBias), m_biasVarMin, m_biasVarMax);
    }
    if (m_hasTemperature)
      m_temperatureAtCal = m_lastTemperature;
  }
  else
  {
    const double tempScale =
        m_hasTemperature ? 1.0 + m_tempScale * std::abs(m_lastTemperature - m_temperatureAtCal)
                         : 1.0;
    for (size_t i = 0; i < 3; ++i)
    {
      m_gyroBiasVar[i] =
          std::clamp(m_gyroBiasVar[i] + tempScale * m_biasQ * dt, m_biasVarMin, m_biasVarMax);
    }
  }

  static std::array<bool, 3> gyroVarInit{{false, false, false}};
  static std::array<bool, 3> accelVarInit{{false, false, false}};
  static std::array<bool, 2> orientVarInit{{false, false}};
  static bool accelMeanStationaryInit = false;
  static Vec3 gyroMean{0.0, 0.0, 0.0};
  static Vec3 accelMeanStationary{0.0, 0.0, 0.0};
  static Vec3 accelResidualMean{0.0, 0.0, 0.0};
  static Vec3 rollPitchMean{0.0, 0.0, 0.0};
  static Vec3 gyroVar{0.0, 0.0, 0.0};
  static Vec3 accelVar{0.0, 0.0, 0.0};
  static Vec3 rollPitchVar{0.0, 0.0, 0.0};

  if (m_isStationary && accelConfidence > 0.7)
  {
    const double alpha = EwmaAlpha(dt, m_ewmaTau);
    if (!accelMeanStationaryInit)
    {
      accelMeanStationary = accel;
      accelMeanStationaryInit = true;
    }
    else
    {
      accelMeanStationary = Add(accelMeanStationary, Scale(Sub(accel, accelMeanStationary), alpha));
    }
    for (size_t i = 0; i < 3; ++i)
    {
      const double accelResidual = accel[i] - accelMeanStationary[i];
      EwmaUpdate(gyroUnbiased[i], alpha, gyroMean[i], gyroVar[i], gyroVarInit[i]);
      EwmaUpdate(accelResidual, alpha, accelResidualMean[i], accelVar[i], accelVarInit[i]);
    }
    const Vec3 euler = EulerFromQuat(q);
    EwmaUpdate(euler[0], alpha, rollPitchMean[0], rollPitchVar[0], orientVarInit[0]);
    EwmaUpdate(euler[1], alpha, rollPitchMean[1], rollPitchVar[1], orientVarInit[1]);

    const bool gyroVarReady = gyroVarInit[0] && gyroVarInit[1] && gyroVarInit[2];
    const bool accelVarReady = accelVarInit[0] && accelVarInit[1] && accelVarInit[2];
    const bool orientVarReady = orientVarInit[0] && orientVarInit[1];
    if (gyroVarReady && accelVarReady && orientVarReady && !m_covarianceInitialized)
    {
      m_covarianceInitialized = true;
      RCLCPP_INFO(get_logger(), "IMU covariance estimates initialized");
    }
    for (size_t i = 0; i < 3; ++i)
    {
      m_gyroVar[i] = std::max(gyroVar[i], 1e-6);
      m_accelVar[i] = std::max(accelVar[i], 1e-6);
    }
    m_rollVar = std::max(rollPitchVar[0], 1e-6);
    m_pitchVar = std::max(rollPitchVar[1], 1e-6);
  }

  // Low-pass gravity for forward inference only to reduce estimator coupling.
  // Gate on low gyro and either not commanded or accel near g for robustness.
  const bool commanded = m_dutyFiltInit;
  const bool gravityLpOk = accelConfidence > 0.7 &&
                           gyroUnbiasedNorm < (m_stationaryGyroThresh * 2.0) &&
                           (!commanded || accelOk);
  if (gravityLpOk)
  {
    const double alpha = EwmaAlpha(dt, m_gravityLpTau);
    if (!m_gravityLpInit)
    {
      m_gravityLpBody = accel;
      m_gravityLpInit = true;
    }
    else
    {
      m_gravityLpBody = Add(m_gravityLpBody, Scale(Sub(accel, m_gravityLpBody), alpha));
    }
  }

  const Vec3 gravityForForward = m_gravityLpInit ? m_gravityLpBody : gravityBodyUpdated;
  const Vec3 accelLinForForward = Sub(accel, gravityForForward);
  Vec3 accelHoriz{0.0, 0.0, 0.0};
  const double gravityNorm = Norm(gravityForForward);
  const bool forwardPlaneOk = gravityNorm > 1e-3;
  if (forwardPlaneOk)
  {
    const Vec3 zHat = Scale(gravityForForward, 1.0 / gravityNorm);
    const double proj = Dot(accelLinForForward, zHat);
    accelHoriz = Sub(accelLinForForward, Scale(zHat, proj));
  }

  if (m_zUpSolved && !m_forwardSolved && forwardPlaneOk)
  {
    const double accelLinXY =
        std::sqrt(accelHoriz[0] * accelHoriz[0] + accelHoriz[1] * accelHoriz[1]);
    const bool event =
        commanded && (std::abs(m_dutyFiltDvdt) > m_dvdtThresh || accelLinXY > m_alinThresh);
    const bool hasResponse =
        accelLinXY > m_forwardResponseAccelThresh || gyroUnbiasedNorm > m_forwardResponseGyroThresh;
    const bool gyroVeto = std::abs(gyroUnbiased[2]) > m_forwardGyroVetoZ ||
                          std::abs(gyroUnbiased[0]) > m_forwardGyroVetoXY ||
                          std::abs(gyroUnbiased[1]) > m_forwardGyroVetoXY;
    // Duty is commanded intent; it may change while a heavy train stays still,
    // so dv/dt is not motion truth.
    if (event && hasResponse && !gyroVeto)
    {
      const double alpha = EwmaAlpha(dt, m_ewmaTau);
      const double sign = (m_dutyFilt >= 0.0) ? 1.0 : -1.0;
      if (std::abs(accelHoriz[0]) >= m_forwardDeadband)
      {
        m_forwardScoreX = (1.0 - alpha) * m_forwardScoreX + alpha * sign * accelHoriz[0];
        m_forwardEnergyX = (1.0 - alpha) * m_forwardEnergyX + alpha * std::abs(accelHoriz[0]);
      }
      if (std::abs(accelHoriz[1]) >= m_forwardDeadband)
      {
        m_forwardScoreY = (1.0 - alpha) * m_forwardScoreY + alpha * sign * accelHoriz[1];
        m_forwardEnergyY = (1.0 - alpha) * m_forwardEnergyY + alpha * std::abs(accelHoriz[1]);
      }

      const double confX = std::abs(m_forwardScoreX) / (m_forwardEnergyX + 1e-6);
      const double confY = std::abs(m_forwardScoreY) / (m_forwardEnergyY + 1e-6);
      int dominantAxis = 0;
      if (confX > confY)
        dominantAxis = 1;
      else if (confY > confX)
        dominantAxis = -1;
      const double dominantConf = dominantAxis == 1 ? confX : confY;
      const double dominantAccel = dominantAxis == 1 ? accelHoriz[0] : accelHoriz[1];
      const bool dominantEligible = dominantAxis != 0 && dominantConf > m_forwardScoreThresh &&
                                    std::abs(dominantAccel) >= m_forwardDeadband;
      if (dominantEligible)
      {
        const int signSample = (sign * dominantAccel >= 0.0) ? 1 : -1;
        if (m_forwardDominantAxis != dominantAxis)
        {
          m_forwardDominantAxis = dominantAxis;
          m_forwardDominanceDuration = 0.0;
          m_forwardSignConsistencyCount = 0;
          m_forwardSignLast = 0;
        }

        if (signSample == m_forwardSignLast)
          m_forwardSignConsistencyCount += 1;
        else
        {
          m_forwardSignLast = signSample;
          m_forwardSignConsistencyCount = 1;
        }

        if (m_forwardSignConsistencyCount >= m_forwardSignConsistencySamples)
          m_forwardDominanceDuration += dt;
      }
      else
      {
        m_forwardDominantAxis = 0;
        m_forwardDominanceDuration = 0.0;
        m_forwardSignConsistencyCount = 0;
        m_forwardSignLast = 0;
      }
    }

    if (m_forwardDominanceDuration >= m_forwardLockSeconds)
    {
      const bool xForward = std::abs(m_forwardScoreX) >= std::abs(m_forwardScoreY);
      const double score = xForward ? m_forwardScoreX : m_forwardScoreY;
      Mapping yawMapping{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
      if (xForward)
      {
        if (score < 0.0)
          yawMapping = {{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}};
      }
      else
      {
        if (score > 0.0)
          yawMapping = {{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}};
        else
          yawMapping = {{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}};
      }

      const Mapping newMapping = MultiplyMapping(yawMapping, m_zUpMapping);
      const Mapping oldMapping = m_activeMapping;
      m_activeMapping = newMapping;
      m_forwardSolved = true;

      const Mapping delta = MultiplyMapping(newMapping, TransposeMapping(oldMapping));
      const Quaternion qDelta = QuatFromRotationMatrix(delta);
      q = MultiplyQuat(q, ConjugateQuat(qDelta));
      q = NormalizeQuat(q);
      m_orientationQuat = {q[0], q[1], q[2], q[3]};
      OnMappingJump("forward-axis lock");
      RCLCPP_INFO(get_logger(), "IMU forward axis locked");
    }
  }

  const double confX = std::abs(m_forwardScoreX) / (m_forwardEnergyX + 1e-6);
  const double confY = std::abs(m_forwardScoreY) / (m_forwardEnergyY + 1e-6);
  if (m_debugPublisher)
  {
    std_msgs::msg::Float64MultiArray debugMsg;
    debugMsg.data = {m_dutyFilt,
                     m_dutyFiltDvdt,
                     accelConfidence,
                     accelHoriz[0],
                     accelHoriz[1],
                     m_forwardScoreX,
                     m_forwardScoreY,
                     m_forwardEnergyX,
                     m_forwardEnergyY,
                     confX,
                     confY,
                     static_cast<double>(m_forwardDominantAxis),
                     m_forwardDominanceDuration};
    m_debugPublisher->publish(debugMsg);
  }
  if (m_statusPublisher && m_statusPublisher->get_subscription_count() > 0)
  {
    std_msgs::msg::Bool statusMsg;
    statusMsg.data = m_forwardSolved;
    m_statusPublisher->publish(statusMsg);
  }
  if (m_mappingDebugPublisher && m_mappingDebugPublisher->get_subscription_count() > 0)
  {
    std_msgs::msg::Float64MultiArray mappingMsg;
    mappingMsg.data.reserve(9);
    for (const auto& row : m_activeMapping)
    {
      for (int value : row)
        mappingMsg.data.push_back(static_cast<double>(value));
    }
    m_mappingDebugPublisher->publish(mappingMsg);
  }

  linearAcceleration.x = accel[0];
  linearAcceleration.y = accel[1];
  linearAcceleration.z = accel[2];

  angularVelocity.x = gyro[0];
  angularVelocity.y = gyro[1];
  angularVelocity.z = gyro[2];

  imuMsg.orientation.w = q[0];
  imuMsg.orientation.x = q[1];
  imuMsg.orientation.y = q[2];
  imuMsg.orientation.z = q[3];

  const double tempScale =
      m_hasTemperature ? 1.0 + m_tempScale * std::abs(m_lastTemperature - m_temperatureAtCal) : 1.0;
  Vec3 accelScaleNoise{0.0, 0.0, 0.0};
  Vec3 gyroScaleNoise{0.0, 0.0, 0.0};
  for (size_t i = 0; i < 3; ++i)
  {
    accelScaleNoise[i] = std::pow(m_accelScaleNoise * std::abs(accel[i]), 2.0) * tempScale;
    gyroScaleNoise[i] = std::pow(m_gyroScaleNoise * std::abs(gyroUnbiased[i]), 2.0) * tempScale;
  }

  const double rollVarPub = m_rollVar / std::max(accelConfidence, 0.1);
  const double pitchVarPub = m_pitchVar / std::max(accelConfidence, 0.1);
  m_yawVar += (m_gyroVar[2] + m_gyroBiasVar[2] + gyroScaleNoise[2]) * dt * dt;
  if (!m_isStationary || accelConfidence < 0.7)
    m_yawVar += m_yawInflateFactor * (1.0 - accelConfidence) * dt;

  // Orientation covariance: roll/pitch from stationary relevel noise, yaw
  // unobservable and inflated by integrated gyro variance + bias drift.
  imuMsg.orientation_covariance = {rollVarPub, 0.0, 0.0, 0.0, pitchVarPub, 0.0, 0.0, 0.0, m_yawVar};

  // Angular velocity covariance: sensor noise plus bias uncertainty (and
  // scale/temperature heuristics).
  imuMsg.angular_velocity_covariance = {
      m_gyroVar[0] + m_gyroBiasVar[0] + gyroScaleNoise[0], 0.0, 0.0, 0.0,
      m_gyroVar[1] + m_gyroBiasVar[1] + gyroScaleNoise[1], 0.0, 0.0, 0.0,
      m_gyroVar[2] + m_gyroBiasVar[2] + gyroScaleNoise[2]};

  // Linear acceleration covariance: specific force (includes gravity) noise,
  // estimated in stationary windows and inflated for dynamic maneuvers.
  const double accelInflate = m_isStationary ? 1.0 : 1.0 / std::max(accelConfidence, 0.1);
  imuMsg.linear_acceleration_covariance = {
      accelInflate * (m_accelVar[0] + accelScaleNoise[0]), 0.0, 0.0, 0.0,
      accelInflate * (m_accelVar[1] + accelScaleNoise[1]), 0.0, 0.0, 0.0,
      accelInflate * (m_accelVar[2] + accelScaleNoise[2])};

  m_publisher->publish(imuMsg);
}
