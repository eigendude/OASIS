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
constexpr double E_MAX = 1.5;
constexpr double IMU_SIGN_CONF_THRESH = 0.4;
constexpr double T_CONFIRM = 0.3;

constexpr double EMA_TAU = 0.2;

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

double EmaAlpha(double dt, double tau)
{
  if (dt <= 0.0)
    return 1.0;
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
  const double accelDev = std::abs(accelMag - GRAVITY);

  const double emaAlpha = EmaAlpha(dt, EMA_TAU);
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
      const double evidence = Clamp(Dot2(aHoriz2d, m_axisHat) / E_MAX, -1.0, 1.0);
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
