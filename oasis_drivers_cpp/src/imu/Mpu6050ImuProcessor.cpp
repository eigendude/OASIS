/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double GRAVITY = 9.80665;
constexpr double EPS = 1e-6;

constexpr double ALPHA_G_STATIONARY = 0.02;
constexpr double ALPHA_G_MOVING = 0.002;

constexpr double BETA_COV = 0.02;
constexpr double BETA_COV_LOCKED = 0.005;
constexpr double A_MOVE_THRESH = 0.25;
constexpr double T_LOCK = 0.6;
constexpr int MIN_COV_SAMPLES = 20;
constexpr double EIGEN_RATIO_THRESH = 3.0;
constexpr double T_RESET = 1.0;

constexpr double GYRO_STILL_THRESH = 0.10;
constexpr double ACCEL_STILL_THRESH = 0.15;
constexpr double BASELINE_UPDATE_DEV_THRESH = 0.25;
constexpr double GYRO_MOVE_THRESH = 0.15;
constexpr double ACCEL_MOVE_THRESH = 0.25;
constexpr double T_STILL = 0.5;
constexpr double T_MOVE = 0.2;

constexpr double DUTY_DEADZONE = 0.05;
constexpr double TAU_CMD = 0.5;
constexpr double CMD_HYST = 0.35;
constexpr double T_GRACE = 0.6;

constexpr double BETA_SIGN = 0.05;
constexpr double BETA_IDLE = 0.005;
constexpr double IMU_SIGN_CONF_THRESH = 0.4;
constexpr double T_CONFIRM = 0.3;

constexpr double EMA_TAU = 0.2;
constexpr double T_GRAVITY_COLLECT = 1.2;
constexpr int MIN_GRAVITY_SAMPLES = 50;
// Bring-up defaults; tighten after field validation.
constexpr double A_DYN_MIN = 0.2;
constexpr int MIN_YAW_SAMPLES = 10;
constexpr double YAW_CONF_THRESH = 0.25;
constexpr double YAW_DYNAMIC_EMA_TAU = 0.3;
constexpr double T_SIGN_FLIP_GUARD = 0.12;

double Clamp(double value, double minValue, double maxValue)
{
  return std::min(std::max(value, minValue), maxValue);
}

double Norm2(const std::array<double, 2>& value)
{
  return std::sqrt(value[0] * value[0] + value[1] * value[1]);
}

double Norm3(const std::array<double, 3>& value)
{
  return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

double Dot2(const std::array<double, 2>& left, const std::array<double, 2>& right)
{
  return left[0] * right[0] + left[1] * right[1];
}

double Dot3(const std::array<double, 3>& left, const std::array<double, 3>& right)
{
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

std::array<double, 2> Normalize2(const std::array<double, 2>& value)
{
  const double norm = Norm2(value);
  if (norm < EPS)
    return {1.0, 0.0};
  return {value[0] / norm, value[1] / norm};
}

std::array<double, 3> Normalize3(const std::array<double, 3>& value)
{
  const double norm = Norm3(value);
  if (norm < EPS)
    return {0.0, 0.0, 1.0};
  return {value[0] / norm, value[1] / norm, value[2] / norm};
}

std::array<double, 3> Cross3(const std::array<double, 3>& left, const std::array<double, 3>& right)
{
  return {left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2],
          left[0] * right[1] - left[1] * right[0]};
}

struct Quaternion
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Quaternion NormalizeQuat(const Quaternion& q)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm < EPS)
    return {};
  return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Quaternion MultiplyQuat(const Quaternion& left, const Quaternion& right)
{
  return {left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
          left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
          left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
          left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
}

std::array<double, 3> RotateVector(const Quaternion& q, const std::array<double, 3>& v)
{
  const Quaternion qv{0.0, v[0], v[1], v[2]};
  const Quaternion qInv{q.w, -q.x, -q.y, -q.z};
  const Quaternion result = MultiplyQuat(MultiplyQuat(q, qv), qInv);
  return {result.x, result.y, result.z};
}

Quaternion QuaternionFromAxisAngle(const std::array<double, 3>& axis, double angle)
{
  const std::array<double, 3> axisHat = Normalize3(axis);
  const double half = 0.5 * angle;
  const double s = std::sin(half);
  return NormalizeQuat({std::cos(half), axisHat[0] * s, axisHat[1] * s, axisHat[2] * s});
}

Quaternion QuaternionFromTwoVectors(const std::array<double, 3>& from,
                                    const std::array<double, 3>& to)
{
  const std::array<double, 3> v0 = Normalize3(from);
  const std::array<double, 3> v1 = Normalize3(to);
  const double dot = Dot3(v0, v1);
  if (dot > 1.0 - 1e-5)
    return {};
  if (dot < -1.0 + 1e-5)
  {
    std::array<double, 3> axis = Cross3({1.0, 0.0, 0.0}, v0);
    if (Norm3(axis) < EPS)
      axis = Cross3({0.0, 1.0, 0.0}, v0);
    return QuaternionFromAxisAngle(axis, M_PI);
  }

  const std::array<double, 3> axis = Cross3(v0, v1);
  const Quaternion q{1.0 + dot, axis[0], axis[1], axis[2]};
  return NormalizeQuat(q);
}

double EmaAlpha(double dt, double tau)
{
  if (dt <= 0.0)
    return 0.0;
  return 1.0 - std::exp(-dt / tau);
}

int SignFromValue(double value)
{
  if (value > 0.0)
    return 1;
  if (value < 0.0)
    return -1;
  return 0;
}
} // namespace

namespace OASIS::IMU
{
Mpu6050ImuProcessor::Mpu6050ImuProcessor()
{
}

Mpu6050ImuProcessor::~Mpu6050ImuProcessor() = default;

void Mpu6050ImuProcessor::SetAccelScale(double accelScale)
{
  m_accelScale = accelScale;
}

void Mpu6050ImuProcessor::SetGyroScale(double gyroScale)
{
  m_gyroScale = gyroScale;
}

double Mpu6050ImuProcessor::GetAccelScale() const
{
  return m_accelScale;
}

double Mpu6050ImuProcessor::GetGyroScale() const
{
  return m_gyroScale;
}

double Mpu6050ImuProcessor::GetAccelMagBaseline() const
{
  return m_hasGMag ? m_gMagEma : 0.0;
}

bool Mpu6050ImuProcessor::HasAccelMagBaseline() const
{
  return m_hasGMag;
}

double Mpu6050ImuProcessor::GetGyroEma() const
{
  return m_gyroEma;
}

double Mpu6050ImuProcessor::GetAccelDevEma() const
{
  return m_accelDevEma;
}

bool Mpu6050ImuProcessor::IsStationary() const
{
  return m_stationary;
}

double Mpu6050ImuProcessor::GetStillTime() const
{
  return m_stillTime;
}

double Mpu6050ImuProcessor::GetMoveTime() const
{
  return m_moveTime;
}

bool Mpu6050ImuProcessor::IsCalibrated() const
{
  return m_calState == CalibrationState::kCalibrated;
}

Mpu6050ImuProcessor::CalibrationState Mpu6050ImuProcessor::GetCalibrationState() const
{
  return m_calState;
}

bool Mpu6050ImuProcessor::HasMountingQuaternion() const
{
  return m_calState == CalibrationState::kCalibrated;
}

geometry_msgs::msg::Quaternion Mpu6050ImuProcessor::GetMountingQuaternionMsg() const
{
  geometry_msgs::msg::Quaternion msg;
  msg.w = m_qMounting[0];
  msg.x = m_qMounting[1];
  msg.y = m_qMounting[2];
  msg.z = m_qMounting[3];
  return msg;
}

std::array<double, 3> Mpu6050ImuProcessor::GetGravityHat() const
{
  return m_gravityHatSolved;
}

std::array<double, 2> Mpu6050ImuProcessor::GetForwardHatIntermediate() const
{
  return m_forwardHatIntermediate;
}

double Mpu6050ImuProcessor::GetYawRadians() const
{
  return m_yawRadians;
}

double Mpu6050ImuProcessor::GetYawConfidence() const
{
  return m_yawConfidence;
}

std::optional<Mpu6050ImuProcessor::ImuOutput> Mpu6050ImuProcessor::Process(
    const Mpu6050ImuProcessor::ImuSample& sample)
{
  double dt = 0.0;
  if (m_hasTimestamp)
  {
    dt = (sample.stamp - m_lastStamp).seconds();
    if (dt < 0.0)
      dt = 0.0;
  }
  m_lastStamp = sample.stamp;
  m_hasTimestamp = true;

  // C) Stationary detection using EMAs of gyro magnitude and accel deviation from gravity.
  const double gyroMag = Norm3(sample.gyro);
  const double accelMag = Norm3(sample.accel);
  const double emaAlpha = EmaAlpha(dt, EMA_TAU);
  if (!m_hasGMag)
  {
    m_gMagEma = accelMag;
    m_hasGMag = true;
  }
  else
  {
    const double accelDevBaseline = std::abs(accelMag - m_gMagEma);
    if (gyroMag < GYRO_STILL_THRESH && accelDevBaseline < BASELINE_UPDATE_DEV_THRESH)
    {
      m_gMagEma = m_gMagEma + emaAlpha * (accelMag - m_gMagEma);
    }
  }
  const double accelDev = std::abs(accelMag - m_gMagEma);

  m_gyroEma = m_gyroEma + emaAlpha * (gyroMag - m_gyroEma);
  m_accelDevEma = m_accelDevEma + emaAlpha * (accelDev - m_accelDevEma);

  if (m_gyroEma < GYRO_STILL_THRESH && m_accelDevEma < ACCEL_STILL_THRESH)
  {
    m_stillTime += dt;
    m_moveTime = 0.0;
  }
  else if (m_gyroEma > GYRO_MOVE_THRESH || m_accelDevEma > ACCEL_MOVE_THRESH)
  {
    m_moveTime += dt;
    m_stillTime = 0.0;
  }

  const bool wasStationary = m_stationary;

  if (m_stationary)
  {
    if (m_moveTime > T_MOVE)
      m_stationary = false;
  }
  else
  {
    if (m_stillTime > T_STILL)
      m_stationary = true;
  }

  if (wasStationary && !m_stationary)
    m_axisResetArmed = true;

  // A) Gravity direction estimation using an EMA that is more aggressive when stationary.
  if (!m_hasGravity)
  {
    m_gravityHat = Normalize3(sample.accel);
    m_hasGravity = true;
  }

  const double alphaG = m_stationary ? ALPHA_G_STATIONARY : ALPHA_G_MOVING;
  std::array<double, 3> gravityBlend{(1.0 - alphaG) * m_gravityHat[0] + alphaG * sample.accel[0],
                                     (1.0 - alphaG) * m_gravityHat[1] + alphaG * sample.accel[1],
                                     (1.0 - alphaG) * m_gravityHat[2] + alphaG * sample.accel[2]};
  m_gravityHat = Normalize3(gravityBlend);

  // A) Horizontal dynamic acceleration: remove gravity, then project onto horizontal plane.
  std::array<double, 3> aDyn{sample.accel[0] - GRAVITY * m_gravityHat[0],
                             sample.accel[1] - GRAVITY * m_gravityHat[1],
                             sample.accel[2] - GRAVITY * m_gravityHat[2]};
  const double aDynDotG = Dot3(aDyn, m_gravityHat);
  std::array<double, 3> aHoriz{aDyn[0] - aDynDotG * m_gravityHat[0],
                               aDyn[1] - aDynDotG * m_gravityHat[1],
                               aDyn[2] - aDynDotG * m_gravityHat[2]};
  const std::array<double, 2> aHoriz2d{aHoriz[0], aHoriz[1]};
  const double aHorizMag = Norm2(aHoriz2d);

  // Calibration state transitions.
  const rclcpp::Logger logger = rclcpp::get_logger("Mpu6050ImuProcessor");
  static rclcpp::Clock clock(RCL_ROS_TIME);

  if (m_calState == CalibrationState::kUncalibrated)
  {
    m_calState = CalibrationState::kGravityCollect;
    m_gravityCollectTime = 0.0;
    m_gravitySamples = 0;
    m_gravitySum = {0.0, 0.0, 0.0};
    RCLCPP_INFO(logger, "IMU calibration: entering gravity collect (stationary window)");
  }

  if (m_calState == CalibrationState::kGravityCollect)
  {
    if (m_stationary)
    {
      m_gravityCollectTime += dt;
      m_gravitySamples += 1;
      m_gravitySum[0] += sample.accel[0];
      m_gravitySum[1] += sample.accel[1];
      m_gravitySum[2] += sample.accel[2];
      if (m_gravityCollectTime >= T_GRAVITY_COLLECT && m_gravitySamples >= MIN_GRAVITY_SAMPLES)
      {
        const std::array<double, 3> meanAccel{
            m_gravitySum[0] / static_cast<double>(m_gravitySamples),
            m_gravitySum[1] / static_cast<double>(m_gravitySamples),
            m_gravitySum[2] / static_cast<double>(m_gravitySamples)};
        m_gravityHatSolved = Normalize3(meanAccel);
        const Quaternion qGravity = QuaternionFromTwoVectors(m_gravityHatSolved, {0.0, 0.0, 1.0});
        m_qGravity = {qGravity.w, qGravity.x, qGravity.y, qGravity.z};
        m_calState = CalibrationState::kGravitySolved;
        RCLCPP_INFO(logger,
                    "IMU calibration: gravity solved g_hat_s=(%.3f, %.3f, %.3f) "
                    "q_gravity=(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                    m_gravityHatSolved[0], m_gravityHatSolved[1], m_gravityHatSolved[2],
                    m_qGravity[0], m_qGravity[1], m_qGravity[2], m_qGravity[3]);
      }
    }
    else if (m_gravityCollectTime > 0.0)
    {
      m_gravityCollectTime = 0.0;
      m_gravitySamples = 0;
      m_gravitySum = {0.0, 0.0, 0.0};
    }
  }

  if (m_calState == CalibrationState::kGravitySolved)
  {
    m_calState = CalibrationState::kYawPulseCollect;
    m_collectingYaw = false;
    m_forwardAccum = {0.0, 0.0};
    m_forwardSamples = 0;
    m_forwardDynamicEma = 0.0;
    m_pulseTime = 0.0;
    m_yawTotalSeen = 0;
    m_yawAccepted = 0;
    m_yawSkipLowDyn = 0;
    m_yawSkipNoSign = 0;
    m_yawSkipSignGuard = 0;
    m_sumHorizMag = 0.0;
    m_yawConfidence = 0.0;
    RCLCPP_INFO(logger, "IMU calibration: waiting for forward-axis lock");
  }

  if (m_calState == CalibrationState::kYawPulseCollect)
  {
    if (m_axisLocked && !m_collectingYaw)
    {
      m_collectingYaw = true;
      m_forwardAccum = {0.0, 0.0};
      m_forwardSamples = 0;
      m_forwardDynamicEma = 0.0;
      m_pulseTime = 0.0;
      m_yawTotalSeen = 0;
      m_yawAccepted = 0;
      m_yawSkipLowDyn = 0;
      m_yawSkipNoSign = 0;
      m_yawSkipSignGuard = 0;
      m_sumHorizMag = 0.0;
      m_yawConfidence = 0.0;
      RCLCPP_INFO(logger, "IMU calibration: forward-axis lock engaged, collecting yaw pulse");
    }

    if (m_axisLocked && m_collectingYaw)
    {
      m_pulseTime += dt;
      m_yawTotalSeen += 1;
      const std::array<double, 3> gHat = m_gravityHatSolved;
      const double aDotG = Dot3(sample.accel, gHat);
      std::array<double, 3> aDynS{sample.accel[0] - aDotG * gHat[0],
                                  sample.accel[1] - aDotG * gHat[1],
                                  sample.accel[2] - aDotG * gHat[2]};
      Quaternion qGravity{m_qGravity[0], m_qGravity[1], m_qGravity[2], m_qGravity[3]};
      const std::array<double, 3> aDynI = RotateVector(qGravity, aDynS);
      std::array<double, 2> horiz{aDynI[0], aDynI[1]};
      const double horizMag = Norm2(horiz);
      const double dynAlpha = EmaAlpha(dt, YAW_DYNAMIC_EMA_TAU);
      m_forwardDynamicEma = m_forwardDynamicEma + dynAlpha * (horizMag - m_forwardDynamicEma);

      if (m_signedAxisSign == 0)
      {
        m_yawSkipNoSign += 1;
      }
      else if (m_timeSinceSignFlip < T_SIGN_FLIP_GUARD)
      {
        m_yawSkipSignGuard += 1;
      }
      else if (horizMag <= A_DYN_MIN)
      {
        m_yawSkipLowDyn += 1;
      }
      else
      {
        m_forwardAccum[0] += static_cast<double>(m_signedAxisSign) * horiz[0];
        m_forwardAccum[1] += static_cast<double>(m_signedAxisSign) * horiz[1];
        m_sumHorizMag += horizMag;
        m_forwardSamples += 1;
        m_yawAccepted += 1;
      }
      const double accumMag = Norm2(m_forwardAccum);
      if (m_sumHorizMag > EPS)
        m_yawConfidence = accumMag / (m_sumHorizMag + EPS);
      else
        m_yawConfidence = 0.0;
    }

    if (!m_axisLocked && m_collectingYaw)
    {
      m_collectingYaw = false;
      const double accumMag = Norm2(m_forwardAccum);
      m_forwardHatIntermediate = Normalize2(m_forwardAccum);
      m_yawRadians = std::atan2(m_forwardHatIntermediate[1], m_forwardHatIntermediate[0]);
      if (m_sumHorizMag > EPS)
        m_yawConfidence = accumMag / (m_sumHorizMag + EPS);
      else
        m_yawConfidence = 0.0;
      const double acceptanceRatio =
          static_cast<double>(m_yawAccepted) / static_cast<double>(std::max(m_yawTotalSeen, 1));
      RCLCPP_INFO(logger,
                  "IMU calibration: yaw pulse end dt=%.3f accum=(%.3f, %.3f) accum_mag=%.3f "
                  "sum_horiz=%.3f dyn_ema=%.3f samples=%d/%d skip_low=%d skip_no_sign=%d "
                  "skip_guard=%d ratio=%.2f yaw=%.3f conf=%.3f thresh(a_dyn=%.2f min_samples=%d "
                  "conf=%.2f)",
                  m_pulseTime, m_forwardAccum[0], m_forwardAccum[1], accumMag, m_sumHorizMag,
                  m_forwardDynamicEma, m_yawAccepted, m_yawTotalSeen, m_yawSkipLowDyn,
                  m_yawSkipNoSign, m_yawSkipSignGuard, acceptanceRatio, m_yawRadians,
                  m_yawConfidence, A_DYN_MIN, MIN_YAW_SAMPLES, YAW_CONF_THRESH);
      if (m_forwardSamples >= MIN_YAW_SAMPLES && m_yawConfidence > YAW_CONF_THRESH)
      {
        const Quaternion qYaw = QuaternionFromAxisAngle({0.0, 0.0, 1.0}, -m_yawRadians);
        const Quaternion qGravity{m_qGravity[0], m_qGravity[1], m_qGravity[2], m_qGravity[3]};
        const Quaternion qMount = MultiplyQuat(qYaw, qGravity);
        const Quaternion qMountNorm = NormalizeQuat(qMount);
        m_qMounting = {qMountNorm.w, qMountNorm.x, qMountNorm.y, qMountNorm.z};
        m_calState = CalibrationState::kCalibrated;
        RCLCPP_INFO(logger,
                    "IMU calibration: yaw solved f_hat_i=(%.3f, %.3f) yaw=%.3f rad conf=%.3f "
                    "q_yaw=(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                    m_forwardHatIntermediate[0], m_forwardHatIntermediate[1], m_yawRadians,
                    m_yawConfidence, qYaw.w, qYaw.x, qYaw.y, qYaw.z);
        RCLCPP_INFO(logger, "IMU calibration: calibrated q_s2t=(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                    m_qMounting[0], m_qMounting[1], m_qMounting[2], m_qMounting[3]);
      }
      else
      {
        RCLCPP_INFO(logger,
                    "IMU calibration: yaw pulse insufficient (samples=%d conf=%.3f dyn=%.3f), "
                    "waiting for next lock",
                    m_forwardSamples, m_yawConfidence, m_forwardDynamicEma);
        m_forwardAccum = {0.0, 0.0};
        m_forwardSamples = 0;
        m_forwardDynamicEma = 0.0;
        m_pulseTime = 0.0;
        m_yawTotalSeen = 0;
        m_yawAccepted = 0;
        m_yawSkipLowDyn = 0;
        m_yawSkipNoSign = 0;
        m_yawSkipSignGuard = 0;
        m_sumHorizMag = 0.0;
      }
    }
  }

  if (m_calState != CalibrationState::kCalibrated)
  {
    const double accumMag = Norm2(m_forwardAccum);
    RCLCPP_INFO_THROTTLE(logger, clock, 2000,
                         "IMU calibration: state=%d stationary=%s still_time=%.2f move_time=%.2f "
                         "gyro_ema=%.3f accel_dev_ema=%.3f axis_locked=%s collecting_yaw=%s "
                         "yaw_seen=%d yaw_accept=%d skip_low=%d skip_no_sign=%d skip_guard=%d "
                         "sum_horiz=%.3f accum_mag=%.3f yaw_dyn=%.3f yaw_conf=%.3f sign_flip_dt=%.3f "
                         "g_hat_s=(%.2f,%.2f,%.2f)",
                         static_cast<int>(m_calState), m_stationary ? "true" : "false", m_stillTime,
                         m_moveTime, m_gyroEma, m_accelDevEma, m_axisLocked ? "true" : "false",
                         m_collectingYaw ? "true" : "false", m_yawTotalSeen, m_yawAccepted,
                         m_yawSkipLowDyn, m_yawSkipNoSign, m_yawSkipSignGuard, m_sumHorizMag,
                         accumMag, m_forwardDynamicEma, m_yawConfidence, m_timeSinceSignFlip,
                         m_gravityHatSolved[0], m_gravityHatSolved[1], m_gravityHatSolved[2]);
  }

  // B) Axis-line estimation reset when stationary long enough.
  if (m_stationary && m_stillTime > T_RESET && m_axisResetArmed)
  {
    m_cov00 = 0.0;
    m_cov01 = 0.0;
    m_cov11 = 0.0;
    m_covSamples = 0;
    m_lockAccumTime = 0.0;
    m_axisLocked = false;
    m_axisHat = {1.0, 0.0};
    m_signConfirmTime = 0.0;
    m_confirmTarget = 0;
    m_axisResetArmed = false;
  }

  // B) Axis-line estimation: signless covariance update in the horizontal plane.
  if (!m_stationary && aHorizMag > A_MOVE_THRESH)
  {
    const double beta = m_axisLocked ? BETA_COV_LOCKED : BETA_COV;
    const double invMag = 1.0 / (aHorizMag + EPS);
    const std::array<double, 2> dir{aHoriz2d[0] * invMag, aHoriz2d[1] * invMag};
    m_cov00 = (1.0 - beta) * m_cov00 + beta * (dir[0] * dir[0]);
    m_cov01 = (1.0 - beta) * m_cov01 + beta * (dir[0] * dir[1]);
    m_cov11 = (1.0 - beta) * m_cov11 + beta * (dir[1] * dir[1]);
    m_covSamples++;
  }

  if (m_covSamples > 0)
  {
    // B) Axis line is the principal eigenvector of the 2x2 covariance.
    const double trace = m_cov00 + m_cov11;
    const double diff = m_cov00 - m_cov11;
    const double disc = std::sqrt(diff * diff + 4.0 * m_cov01 * m_cov01);
    const double lambda1 = 0.5 * (trace + disc);
    const double lambda2 = 0.5 * (trace - disc);
    const double ratio = lambda1 / std::max(lambda2, EPS);

    std::array<double, 2> axisCandidate{1.0, 0.0};
    if (std::abs(m_cov01) > EPS)
    {
      axisCandidate = {lambda1 - m_cov11, m_cov01};
    }
    else
    {
      axisCandidate =
          (m_cov00 >= m_cov11) ? std::array<double, 2>{1.0, 0.0} : std::array<double, 2>{0.0, 1.0};
    }
    axisCandidate = Normalize2(axisCandidate);

    if (Dot2(axisCandidate, m_axisHat) < 0.0)
    {
      axisCandidate[0] = -axisCandidate[0];
      axisCandidate[1] = -axisCandidate[1];
    }

    m_axisHat = axisCandidate;

    if (!m_axisLocked && ratio > EIGEN_RATIO_THRESH && m_covSamples >= MIN_COV_SAMPLES &&
        !m_stationary)
    {
      m_lockAccumTime += dt;
      if (m_lockAccumTime > T_LOCK)
        m_axisLocked = true;
    }
    else if (!m_axisLocked)
    {
      m_lockAccumTime = 0.0;
    }
  }

  // D) Command sign filtering: deadzone + first-order lag + hysteresis.
  int cmdSignRaw = 0;
  if (sample.duty_cycle > DUTY_DEADZONE)
    cmdSignRaw = 1;
  else if (sample.duty_cycle < -DUTY_DEADZONE)
    cmdSignRaw = -1;

  if (TAU_CMD > EPS && dt > 0.0)
  {
    m_cmdFiltered += (dt / TAU_CMD) * (static_cast<double>(cmdSignRaw) - m_cmdFiltered);
    m_cmdFiltered = Clamp(m_cmdFiltered, -1.0, 1.0);
  }
  else
  {
    m_cmdFiltered = static_cast<double>(cmdSignRaw);
  }

  int cmdSign = 0;
  if (m_cmdFiltered > CMD_HYST)
    cmdSign = 1;
  else if (m_cmdFiltered < -CMD_HYST)
    cmdSign = -1;

  const int prevCmdSign = m_cmdSign;
  if (cmdSign != m_cmdSign && cmdSign != 0 && m_cmdSign != 0)
  {
    m_reversalGraceTime = T_GRACE;
    m_signConfirmTime = 0.0;
    m_confirmTarget = 0;
  }
  m_cmdSign = cmdSign;

  if (m_reversalGraceTime > 0.0)
  {
    m_reversalGraceTime = std::max(0.0, m_reversalGraceTime - dt);
    m_signConfirmTime = 0.0;
    m_confirmTarget = 0;
  }
  else if (prevCmdSign == 0 && m_cmdSign != 0)
  {
    m_signedAxisSign = m_cmdSign;
    m_signConfirmTime = 0.0;
    m_confirmTarget = 0;
  }

  // E) IMU motion sign evidence along the locked axis.
  if (m_axisLocked)
  {
    if (!m_stationary && aHorizMag > A_MOVE_THRESH)
    {
      const double invMag = 1.0 / (aHorizMag + EPS);
      const std::array<double, 2> dir{aHoriz2d[0] * invMag, aHoriz2d[1] * invMag};
      const double evidence = Dot2(dir, m_axisHat);
      m_imuEvidence = (1.0 - BETA_SIGN) * m_imuEvidence + BETA_SIGN * evidence;
    }
    else
    {
      m_imuEvidence = (1.0 - BETA_IDLE) * m_imuEvidence;
    }
  }
  else
  {
    m_imuEvidence = (1.0 - BETA_IDLE) * m_imuEvidence;
  }

  const bool imuStrong = std::abs(m_imuEvidence) > IMU_SIGN_CONF_THRESH;
  const int imuSign = SignFromValue(m_imuEvidence);

  // F) Signed axis selection: duty sign primary, IMU evidence confirms or overrides.
  if (m_reversalGraceTime <= 0.0)
  {
    if (m_cmdSign != 0)
    {
      if (imuStrong && imuSign != 0 && imuSign != m_cmdSign)
      {
        if (imuSign != m_confirmTarget)
        {
          m_confirmTarget = imuSign;
          m_signConfirmTime = 0.0;
        }
        m_signConfirmTime += dt;
        if (m_signConfirmTime >= T_CONFIRM)
          m_signedAxisSign = imuSign;
      }
      else
      {
        m_signedAxisSign = m_cmdSign;
        m_signConfirmTime = 0.0;
        m_confirmTarget = 0;
      }
    }
    else
    {
      if (imuStrong)
      {
        if (imuSign != m_confirmTarget)
        {
          m_confirmTarget = imuSign;
          m_signConfirmTime = 0.0;
        }
        m_signConfirmTime += dt;
        if (m_signConfirmTime >= T_CONFIRM)
          m_signedAxisSign = imuSign;
      }
      else
      {
        m_signConfirmTime = 0.0;
        m_confirmTarget = 0;
      }
    }
  }

  if (m_signedAxisSign != m_prevSignedAxisSign)
  {
    m_timeSinceSignFlip = 0.0;
    m_prevSignedAxisSign = m_signedAxisSign;
  }
  else
  {
    m_timeSinceSignFlip += dt;
  }

  if (m_axisLocked != m_prevAxisLocked)
  {
    RCLCPP_INFO(logger,
                "IMU calibration: axis lock %s t=%.3f state=%d still_time=%.2f move_time=%.2f "
                "signed_sign=%d gyro_ema=%.3f accel_dev_ema=%.3f",
                m_axisLocked ? "engaged" : "released", sample.stamp.seconds(),
                static_cast<int>(m_calState), m_stillTime, m_moveTime, m_signedAxisSign, m_gyroEma,
                m_accelDevEma);
    m_prevAxisLocked = m_axisLocked;
  }

  ImuOutput output;
  output.linear_acceleration = sample.accel;
  output.angular_velocity = sample.gyro;
  output.u_hat2d = m_axisHat;
  output.u_signed2d = {m_signedAxisSign * m_axisHat[0], m_signedAxisSign * m_axisHat[1]};
  output.axis_locked = m_axisLocked;
  output.cmd_sign = m_cmdSign;
  output.signed_sign = m_signedAxisSign;
  output.stationary = m_stationary;

  return output;
}
} // namespace OASIS::IMU
