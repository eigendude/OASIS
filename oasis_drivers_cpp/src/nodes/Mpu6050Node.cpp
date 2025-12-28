/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Mpu6050Node.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <I2Cdev.h>
#include <rclcpp/rclcpp.hpp>

namespace
{

// Default node name
constexpr const char* NODE_NAME = "mpu6050_imu_driver";

// ROS topics
constexpr const char* IMU_TOPIC = "imu";
constexpr const char* CONDUCTOR_STATE_TOPIC = "conductor_state";

// ROS frame IDs
constexpr const char* FRAME_ID = "imu_link";

// ROS parameters
constexpr const char* DEFAULT_I2C_DEVICE = "/dev/i2c-1";
constexpr double DEFAULT_PUBLISH_RATE_HZ = 50.0;

// IMU parameters
constexpr double GRAVITY = 9.80665; // m/s^2
constexpr double ACCEL_SCALE = GRAVITY / 16384.0; // +/-2g full scale
constexpr double GYRO_SCALE = (M_PI / 180.0) / 131.0; // +/-250 deg/s full scale
constexpr double Z_UP_SCORE_GRAVITY_PENALTY = 1.0;

using Vec3 = OASIS::ROS::Mpu6050Node::Vec3;
using Mapping = OASIS::ROS::Mpu6050Node::Mapping;
using Quaternion = OASIS::ROS::Mpu6050Node::Quaternion;

// Utility helpers

double AccelScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_ACCEL_FS_2:
      return GRAVITY / 16384.0;
    case MPU6050_ACCEL_FS_4:
      return GRAVITY / 8192.0;
    case MPU6050_ACCEL_FS_8:
      return GRAVITY / 4096.0;
    case MPU6050_ACCEL_FS_16:
      return GRAVITY / 2048.0;
    default:
      return ACCEL_SCALE;
  }
}

double GyroScaleFromRange(uint8_t range)
{
  switch (range)
  {
    case MPU6050_GYRO_FS_250:
      return (M_PI / 180.0) / 131.0;
    case MPU6050_GYRO_FS_500:
      return (M_PI / 180.0) / 65.5;
    case MPU6050_GYRO_FS_1000:
      return (M_PI / 180.0) / 32.8;
    case MPU6050_GYRO_FS_2000:
      return (M_PI / 180.0) / 16.4;
    default:
      return GYRO_SCALE;
  }
}

Vec3 Add(const Vec3& a, const Vec3& b)
{
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 Sub(const Vec3& a, const Vec3& b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 Scale(const Vec3& a, double s)
{
  return {a[0] * s, a[1] * s, a[2] * s};
}

double Dot(const Vec3& a, const Vec3& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 Cross(const Vec3& a, const Vec3& b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double Norm(const Vec3& a)
{
  return std::sqrt(Dot(a, a));
}

Vec3 Normalize(const Vec3& a)
{
  const double n = Norm(a);
  if (n <= 1e-9)
    return {0.0, 0.0, 0.0};
  return Scale(a, 1.0 / n);
}

Vec3 ApplyMapping(const Mapping& mapping, const Vec3& v)
{
  Vec3 out{0.0, 0.0, 0.0};
  for (size_t i = 0; i < 3; ++i)
  {
    double sum = 0.0;
    for (size_t j = 0; j < 3; ++j)
      sum += static_cast<double>(mapping[i][j]) * v[j];
    out[i] = sum;
  }
  return out;
}

Mapping MultiplyMapping(const Mapping& a, const Mapping& b)
{
  Mapping out{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      int sum = 0;
      for (size_t k = 0; k < 3; ++k)
        sum += a[i][k] * b[k][j];
      out[i][j] = sum;
    }
  }
  return out;
}

Mapping TransposeMapping(const Mapping& m)
{
  Mapping out{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
      out[i][j] = m[j][i];
  }
  return out;
}

Quaternion NormalizeQuat(const Quaternion& q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n <= 1e-9)
    return {1.0, 0.0, 0.0, 0.0};
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Quaternion MultiplyQuat(const Quaternion& a, const Quaternion& b)
{
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

Quaternion ConjugateQuat(const Quaternion& q)
{
  return {q[0], -q[1], -q[2], -q[3]};
}

Vec3 RotateVec(const Quaternion& q, const Vec3& v)
{
  const Quaternion vq{0.0, v[0], v[1], v[2]};
  const Quaternion rq = MultiplyQuat(MultiplyQuat(q, vq), ConjugateQuat(q));
  return {rq[1], rq[2], rq[3]};
}

Quaternion QuatFromAxisAngle(const Vec3& axis, double angle)
{
  const double half = angle * 0.5;
  const double s = std::sin(half);
  return NormalizeQuat({std::cos(half), axis[0] * s, axis[1] * s, axis[2] * s});
}

Quaternion QuatFromEuler(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}

Quaternion Slerp(const Quaternion& a, const Quaternion& b, double t)
{
  if (t <= 0.0)
    return a;
  if (t >= 1.0)
    return b;
  double cosTheta = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  Quaternion bb = b;
  if (cosTheta < 0.0)
  {
    cosTheta = -cosTheta;
    bb = {-b[0], -b[1], -b[2], -b[3]};
  }
  if (cosTheta > 0.9995)
  {
    const Quaternion out{a[0] + t * (bb[0] - a[0]), a[1] + t * (bb[1] - a[1]),
                         a[2] + t * (bb[2] - a[2]), a[3] + t * (bb[3] - a[3])};
    return NormalizeQuat(out);
  }
  const double angle = std::acos(std::clamp(cosTheta, -1.0, 1.0));
  const double s = std::sin(angle);
  const double wa = std::sin((1.0 - t) * angle) / s;
  const double wb = std::sin(t * angle) / s;
  return {a[0] * wa + bb[0] * wb, a[1] * wa + bb[1] * wb, a[2] * wa + bb[2] * wb,
          a[3] * wa + bb[3] * wb};
}

Vec3 EulerFromQuat(const Quaternion& q)
{
  const double sinr = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  const double cosr = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
  const double roll = std::atan2(sinr, cosr);
  const double sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  const double siny = 2.0 * (q[0] * q[3] + q[1] * q[2]);
  const double cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
  const double yaw = std::atan2(siny, cosy);
  return {roll, pitch, yaw};
}

Quaternion QuatFromRotationMatrix(const Mapping& m)
{
  const double trace = static_cast<double>(m[0][0] + m[1][1] + m[2][2]);
  Quaternion q{1.0, 0.0, 0.0, 0.0};
  if (trace > 0.0)
  {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q[0] = 0.25 * s;
    q[1] = (m[2][1] - m[1][2]) / s;
    q[2] = (m[0][2] - m[2][0]) / s;
    q[3] = (m[1][0] - m[0][1]) / s;
  }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
  {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q[0] = (m[2][1] - m[1][2]) / s;
    q[1] = 0.25 * s;
    q[2] = (m[0][1] + m[1][0]) / s;
    q[3] = (m[0][2] + m[2][0]) / s;
  }
  else if (m[1][1] > m[2][2])
  {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    q[0] = (m[0][2] - m[2][0]) / s;
    q[1] = (m[0][1] + m[1][0]) / s;
    q[2] = 0.25 * s;
    q[3] = (m[1][2] + m[2][1]) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    q[0] = (m[1][0] - m[0][1]) / s;
    q[1] = (m[0][2] + m[2][0]) / s;
    q[2] = (m[1][2] + m[2][1]) / s;
    q[3] = 0.25 * s;
  }
  return NormalizeQuat(q);
}

std::vector<Mapping> GenerateMappings()
{
  std::vector<Mapping> mappings;
  std::array<int, 3> perm{{0, 1, 2}};
  auto parity = [](const std::array<int, 3>& p)
  {
    int inv = 0;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = i + 1; j < 3; ++j)
      {
        if (p[i] > p[j])
          ++inv;
      }
    }
    return (inv % 2 == 0) ? 1 : -1;
  };
  do
  {
    const int permParity = parity(perm);
    for (int sx : {-1, 1})
    {
      for (int sy : {-1, 1})
      {
        for (int sz : {-1, 1})
        {
          const int det = permParity * sx * sy * sz;
          if (det != 1)
            continue;
          Mapping m{{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
          m[0][perm[0]] = sx;
          m[1][perm[1]] = sy;
          m[2][perm[2]] = sz;
          mappings.push_back(m);
        }
      }
    }
  } while (std::next_permutation(perm.begin(), perm.end()));
  return mappings;
}

double EwmaAlpha(double dt, double tau)
{
  if (tau <= 1e-6)
    return 1.0;
  return std::clamp(dt / (tau + dt), 0.0, 1.0);
}

void EwmaUpdate(double x, double alpha, double& mean, double& var, bool& initialized)
{
  if (!initialized)
  {
    mean = x;
    var = 0.0;
    initialized = true;
    return;
  }
  const double delta = x - mean;
  mean += alpha * delta;
  var = (1.0 - alpha) * (var + alpha * delta * delta);
}

} // namespace

using namespace OASIS::ROS;
using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node() : rclcpp::Node(NODE_NAME)
{
  declare_parameter("i2c_device", std::string(DEFAULT_I2C_DEVICE));
  declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  declare_parameter("conductor_state_topic", std::string(CONDUCTOR_STATE_TOPIC));
  declare_parameter("stationary_voltage_thresh", m_stationaryVoltageThresh);
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
  declare_parameter("motor_voltage_lp_tau", m_motorVoltageLpTau);
  declare_parameter("dvdt_thresh", m_dvdtThresh);
  declare_parameter("alin_thresh", m_alinThresh);
  declare_parameter("forward_candidate_window_seconds", m_forwardCandidateWindowSeconds);
  declare_parameter("forward_response_accel_thresh", m_forwardResponseAccelThresh);
  declare_parameter("forward_response_gyro_thresh", m_forwardResponseGyroThresh);
  declare_parameter("forward_lock_seconds", m_forwardLockSeconds);
  declare_parameter("forward_score_thresh", m_forwardScoreThresh);
  declare_parameter("yaw_inflate_factor", m_yawInflateFactor);
  declare_parameter("dt_min", m_dtMin);
  declare_parameter("dt_max", m_dtMax);

  m_i2cDevice = get_parameter("i2c_device").as_string();
  m_conductorStateTopic = get_parameter("conductor_state_topic").as_string();
  m_stationaryVoltageThresh = get_parameter("stationary_voltage_thresh").as_double();
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
  m_motorVoltageLpTau = get_parameter("motor_voltage_lp_tau").as_double();
  m_dvdtThresh = get_parameter("dvdt_thresh").as_double();
  m_alinThresh = get_parameter("alin_thresh").as_double();
  m_forwardCandidateWindowSeconds = get_parameter("forward_candidate_window_seconds").as_double();
  m_forwardResponseAccelThresh = get_parameter("forward_response_accel_thresh").as_double();
  m_forwardResponseGyroThresh = get_parameter("forward_response_gyro_thresh").as_double();
  m_forwardLockSeconds = get_parameter("forward_lock_seconds").as_double();
  m_forwardScoreThresh = get_parameter("forward_score_thresh").as_double();
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

  const rclcpp::Time msgStamp(msg.header.stamp);

  if (m_lastMotorStamp.nanoseconds() > 0)
  {
    const double dt = (msgStamp - m_lastMotorStamp).seconds();
    if (dt > 1e-3)
    {
      m_motorVoltageDvdt = (msg.motor_voltage - m_motorVoltage) / dt;
      const double alpha = EwmaAlpha(dt, m_motorVoltageLpTau);
      const double prevFilt = m_motorVoltageFilt;
      if (!m_motorVoltageFiltInit)
      {
        m_motorVoltageFilt = msg.motor_voltage;
        m_motorVoltageFiltDvdt = 0.0;
        m_motorVoltageFiltInit = true;
      }
      else
      {
        m_motorVoltageFilt += alpha * (msg.motor_voltage - m_motorVoltageFilt);
        m_motorVoltageFiltDvdt = (m_motorVoltageFilt - prevFilt) / dt;
      }
    }
  }
  else
  {
    m_motorVoltageFilt = msg.motor_voltage;
    m_motorVoltageFiltDvdt = 0.0;
    m_motorVoltageFiltInit = true;
  }
  m_motorVoltage = msg.motor_voltage;
  m_lastMotorStamp = msgStamp;
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
  geometry_msgs::msg::Vector3& linearAceleration = imuMsg.linear_acceleration;

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
  const Vec3 gravityNorm = Normalize(gravityBody);
  const Vec3 error = Cross(gravityNorm, accelNorm);

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
  static bool orientVarInit = false;
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
    EwmaUpdate(euler[0], alpha, rollPitchMean[0], rollPitchVar[0], orientVarInit);
    EwmaUpdate(euler[1], alpha, rollPitchMean[1], rollPitchVar[1], orientVarInit);

    const bool gyroVarReady = gyroVarInit[0] && gyroVarInit[1] && gyroVarInit[2];
    const bool accelVarReady = accelVarInit[0] && accelVarInit[1] && accelVarInit[2];
    if (gyroVarReady && accelVarReady && orientVarInit && !m_covarianceInitialized)
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
  // Gate on low gyro and either commanded idle intent or accel near g for robustness.
  const bool commanded = std::abs(m_motorVoltageFilt) > m_stationaryVoltageThresh;
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

  if (m_zUpSolved && !m_forwardSolved)
  {
    const double accelLinXY = std::sqrt(accelLinForForward[0] * accelLinForForward[0] +
                                        accelLinForForward[1] * accelLinForForward[1]);
    m_forwardCandidateTime = std::max(0.0, m_forwardCandidateTime - dt);
    if (std::abs(m_motorVoltageFiltDvdt) > m_dvdtThresh && commanded)
      m_forwardCandidateTime = m_forwardCandidateWindowSeconds;
    const bool event = m_forwardCandidateTime > 0.0 || accelLinXY > m_alinThresh;
    const bool hasResponse =
        accelLinXY > m_forwardResponseAccelThresh || gyroUnbiasedNorm > m_forwardResponseGyroThresh;
    // Motor voltage is commanded intent; dv/dt alone is not motion truth.
    if (event && commanded && hasResponse)
    {
      const double alpha = EwmaAlpha(dt, m_ewmaTau);
      const double sign = (m_motorVoltageFilt >= 0.0) ? 1.0 : -1.0;
      m_forwardScoreX = (1.0 - alpha) * m_forwardScoreX + alpha * sign * accelLinForForward[0];
      m_forwardScoreY = (1.0 - alpha) * m_forwardScoreY + alpha * sign * accelLinForForward[1];

      const double absX = std::abs(m_forwardScoreX);
      const double absY = std::abs(m_forwardScoreY);
      int dominantAxis = 0;
      if (absX > absY)
        dominantAxis = 1;
      else if (absY > absX)
        dominantAxis = -1;
      if (dominantAxis != 0 && std::max(absX, absY) > m_forwardScoreThresh)
      {
        if (m_forwardDominantAxis == 0 || m_forwardDominantAxis == dominantAxis)
        {
          m_forwardDominantAxis = dominantAxis;
          m_forwardDominanceDuration += dt;
        }
        else
        {
          m_forwardDominantAxis = dominantAxis;
          m_forwardDominanceDuration = 0.0;
        }
      }
      else
      {
        m_forwardDominantAxis = 0;
        m_forwardDominanceDuration = 0.0;
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
      RCLCPP_INFO(get_logger(), "IMU forward axis locked");
    }
  }

  linearAceleration.x = accel[0];
  linearAceleration.y = accel[1];
  linearAceleration.z = accel[2];

  angularVelocity.x = gyro[0];
  angularVelocity.y = gyro[1];
  angularVelocity.z = gyro[2];

  imuMsg.orientation.w = q[0];
  imuMsg.orientation.x = q[1];
  imuMsg.orientation.y = q[2];
  imuMsg.orientation.z = q[3];

  const double tempScale =
      m_hasTemperature ? 1.0 + m_tempScale * std::abs(m_lastTemperature - m_temperatureAtCal) : 1.0;
  const double accelScaleNoise = std::pow(m_accelScaleNoise * Norm(accel), 2.0) * tempScale;
  const double gyroScaleNoise = std::pow(m_gyroScaleNoise * Norm(gyroUnbiased), 2.0) * tempScale;

  const double rollVarPub = m_rollVar / std::max(accelConfidence, 0.1);
  const double pitchVarPub = m_pitchVar / std::max(accelConfidence, 0.1);
  m_yawVar += (m_gyroVar[2] + m_gyroBiasVar[2] + gyroScaleNoise) * dt * dt;
  if (!m_isStationary || accelConfidence < 0.7)
    m_yawVar += m_yawInflateFactor * (1.0 - accelConfidence) * dt;

  // Orientation covariance: roll/pitch from stationary relevel noise, yaw
  // unobservable and inflated by integrated gyro variance + bias drift.
  imuMsg.orientation_covariance = {rollVarPub, 0.0, 0.0, 0.0, pitchVarPub, 0.0, 0.0, 0.0, m_yawVar};

  // Angular velocity covariance: sensor noise plus bias uncertainty (and
  // scale/temperature heuristics).
  imuMsg.angular_velocity_covariance = {
      m_gyroVar[0] + m_gyroBiasVar[0] + gyroScaleNoise, 0.0, 0.0, 0.0,
      m_gyroVar[1] + m_gyroBiasVar[1] + gyroScaleNoise, 0.0, 0.0, 0.0,
      m_gyroVar[2] + m_gyroBiasVar[2] + gyroScaleNoise};

  // Linear acceleration covariance: specific force (includes gravity) noise,
  // estimated in stationary windows and inflated for dynamic maneuvers.
  const double accelInflate = m_isStationary ? 1.0 : 1.0 / std::max(accelConfidence, 0.1);
  imuMsg.linear_acceleration_covariance = {
      accelInflate * (m_accelVar[0] + accelScaleNoise), 0.0, 0.0, 0.0,
      accelInflate * (m_accelVar[1] + accelScaleNoise), 0.0, 0.0, 0.0,
      accelInflate * (m_accelVar[2] + accelScaleNoise)};

  m_publisher->publish(imuMsg);
}
