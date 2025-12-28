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
#include <utility>

namespace OASIS::IMU
{
using namespace OASIS::ROS::Mpu6050NodeUtils;

Mpu6050ImuProcessor::Mpu6050ImuProcessor(Config config) : m_config(std::move(config))
{
  m_mappings = GenerateMappings();
}

void Mpu6050ImuProcessor::SetAccelScale(double accel_scale)
{
  m_accelScale = accel_scale;
}

void Mpu6050ImuProcessor::SetGyroScale(double gyro_scale)
{
  m_gyroScale = gyro_scale;
}

void Mpu6050ImuProcessor::RegisterMappingJump(const char* reason, Output& output)
{
  m_yawVar = std::max(m_yawVar, 1.5);
  if (!output.mapping_jump_reason.has_value())
    output.mapping_jump_reason = reason;
}

std::optional<Mpu6050ImuProcessor::Output> Mpu6050ImuProcessor::Process(
    const RawSample& sample,
    const MotorIntentInput& motor)
{
  if (sample.temp_raw.has_value())
  {
    m_hasTemperature = true;
    m_lastTemperature = static_cast<double>(*sample.temp_raw) / 340.0 + 36.53;
  }

  if (m_lastTimestamp == 0.0)
  {
    m_lastTimestamp = sample.timestamp;
    return std::nullopt;
  }

  const double dt = sample.timestamp - m_lastTimestamp;
  m_lastTimestamp = sample.timestamp;
  if (dt < m_config.dt_min || dt > m_config.dt_max)
    return std::nullopt;

  Output output;

  const Vec3 accelRaw = {static_cast<double>(sample.ax) * m_accelScale,
                         static_cast<double>(sample.ay) * m_accelScale,
                         static_cast<double>(sample.az) * m_accelScale};
  const Vec3 gyroRaw = {static_cast<double>(sample.gx) * m_gyroScale,
                        static_cast<double>(sample.gy) * m_gyroScale,
                        static_cast<double>(sample.gz) * m_gyroScale};

  const Vec3 gyroMapped = ApplyMapping(m_activeMapping, gyroRaw);
  const Vec3 gyroUnbiasedPreSolve = Sub(gyroMapped, m_gyroBias);
  const double gyroMagPreSolve = Norm(gyroUnbiasedPreSolve);

  if (!m_zUpSolved)
  {
    const double accelMag = Norm(accelRaw);
    const bool accelOk = std::abs(accelMag - GRAVITY) < m_config.stationary_accel_mag_thresh;
    if (gyroMagPreSolve < m_config.stationary_gyro_thresh && accelOk)
    {
      const double alpha = EwmaAlpha(dt, m_config.ewma_tau);
      if (!m_accelMeanInit)
      {
        m_accelMean = accelRaw;
        m_accelMeanInit = true;
      }
      else
      {
        m_accelMean = Add(m_accelMean, Scale(Sub(accelRaw, m_accelMean), alpha));
      }
      m_stationaryGoodDuration += dt;
    }
    else
    {
      m_stationaryGoodDuration = 0.0;
    }

    if (m_accelMeanInit && m_stationaryGoodDuration >= m_config.stationary_hold_seconds)
    {
      double bestScore = -1e9;
      Mapping best = m_zUpMapping;
      for (const auto& mapping : m_mappings)
      {
        const Vec3 mapped = ApplyMapping(mapping, m_accelMean);
        const double score = mapped[2] - 0.5 * (std::abs(mapped[0]) + std::abs(mapped[1])) -
                             Z_UP_SCORE_GRAVITY_PENALTY * std::abs(mapped[2] - GRAVITY);
        if (score > bestScore)
        {
          bestScore = score;
          best = mapping;
        }
      }
      const Vec3 mappedBest = ApplyMapping(best, m_accelMean);
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
        RegisterMappingJump("Z-up solve", output);
        output.info_messages.emplace_back("IMU mapping leveled to +Z");
      }
    }
  }

  Vec3 accel = ApplyMapping(m_activeMapping, accelRaw);
  Vec3 gyro = ApplyMapping(m_activeMapping, gyroRaw);
  const Vec3 gyroUnbiased = Sub(gyro, m_gyroBias);

  const double accelMag = Norm(accel);
  const double gyroUnbiasedNorm = Norm(gyroUnbiased);
  const bool accelOk = std::abs(accelMag - GRAVITY) < m_config.stationary_accel_mag_thresh;
  const bool gyroOk = gyroUnbiasedNorm < m_config.stationary_gyro_thresh;
  const bool stationaryNow = accelOk && gyroOk;
  if (stationaryNow)
  {
    m_stationaryDuration += dt;
    if (!m_isStationary && m_stationaryDuration >= m_config.stationary_hold_seconds)
    {
      m_isStationary = true;
      output.info_messages.emplace_back("IMU stationary detected");
    }
  }
  else
  {
    if (m_isStationary)
      output.info_messages.emplace_back("IMU moving detected");
    m_isStationary = false;
    m_stationaryDuration = 0.0;
  }

  const double accelConfidence =
      std::clamp(1.0 - std::abs(accelMag - GRAVITY) / m_config.accel_confidence_range, 0.0, 1.0);

  Quaternion q = {m_orientationQuat[0], m_orientationQuat[1], m_orientationQuat[2],
                  m_orientationQuat[3]};
  const Vec3 gravityBody = RotateVec(ConjugateQuat(q), {0.0, 0.0, GRAVITY});
  const Vec3 accelNorm = accelMag > 1e-6 ? Scale(accel, 1.0 / accelMag) : Vec3{0.0, 0.0, 0.0};
  const Vec3 gravityDir = Normalize(gravityBody);
  const Vec3 error = Cross(gravityDir, accelNorm);

  const double kpEff = m_config.mahony_kp * accelConfidence;
  const double kiEff = m_config.mahony_ki * accelConfidence;

  m_mahonyIntegral = Add(m_mahonyIntegral, Scale(error, kiEff * dt));
  for (size_t i = 0; i < 3; ++i)
    m_mahonyIntegral[i] =
        std::clamp(m_mahonyIntegral[i], -m_config.mahony_integral_limit,
                   m_config.mahony_integral_limit);
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
    q = Slerp(q, target, std::clamp(m_config.relevel_rate * dt, 0.0, 1.0));
  }

  m_orientationQuat = {q[0], q[1], q[2], q[3]};

  const Vec3 gravityBodyUpdated = RotateVec(ConjugateQuat(q), {0.0, 0.0, GRAVITY});

  if (m_isStationary && accelConfidence > 0.7)
  {
    const double alphaBias = EwmaAlpha(dt, m_config.bias_tau);
    m_gyroBias = Add(m_gyroBias, Scale(Sub(gyro, m_gyroBias), alphaBias));
    for (size_t i = 0; i < 3; ++i)
    {
      m_gyroBiasVar[i] =
          std::clamp(m_gyroBiasVar[i] * (1.0 - alphaBias), m_config.bias_var_min,
                     m_config.bias_var_max);
    }
    if (m_hasTemperature)
      m_temperatureAtCal = m_lastTemperature;
  }
  else
  {
    const double tempScale =
        m_hasTemperature ? 1.0 + m_config.temp_scale * std::abs(m_lastTemperature - m_temperatureAtCal)
                         : 1.0;
    for (size_t i = 0; i < 3; ++i)
    {
      m_gyroBiasVar[i] =
          std::clamp(m_gyroBiasVar[i] + tempScale * m_config.bias_q * dt, m_config.bias_var_min,
                     m_config.bias_var_max);
    }
  }

  if (m_isStationary && accelConfidence > 0.7)
  {
    const double alpha = EwmaAlpha(dt, m_config.ewma_tau);
    if (!m_accelMeanStationaryInit)
    {
      m_accelMeanStationary = accel;
      m_accelMeanStationaryInit = true;
    }
    else
    {
      m_accelMeanStationary =
          Add(m_accelMeanStationary, Scale(Sub(accel, m_accelMeanStationary), alpha));
    }
    for (size_t i = 0; i < 3; ++i)
    {
      const double accelResidual = accel[i] - m_accelMeanStationary[i];
      EwmaUpdate(gyroUnbiased[i], alpha, m_gyroMean[i], m_gyroVar[i], m_gyroVarInit[i]);
      EwmaUpdate(accelResidual, alpha, m_accelResidualMean[i], m_accelVar[i],
                 m_accelVarInit[i]);
    }
    const Vec3 euler = EulerFromQuat(q);
    EwmaUpdate(euler[0], alpha, m_rollPitchMean[0], m_rollPitchVar[0], m_orientVarInit[0]);
    EwmaUpdate(euler[1], alpha, m_rollPitchMean[1], m_rollPitchVar[1], m_orientVarInit[1]);

    const bool gyroVarReady = m_gyroVarInit[0] && m_gyroVarInit[1] && m_gyroVarInit[2];
    const bool accelVarReady = m_accelVarInit[0] && m_accelVarInit[1] && m_accelVarInit[2];
    const bool orientVarReady = m_orientVarInit[0] && m_orientVarInit[1];
    if (gyroVarReady && accelVarReady && orientVarReady && !m_covarianceInitialized)
    {
      m_covarianceInitialized = true;
      output.info_messages.emplace_back("IMU covariance estimates initialized");
    }
    for (size_t i = 0; i < 3; ++i)
    {
      m_gyroVar[i] = std::max(m_gyroVar[i], 1e-6);
      m_accelVar[i] = std::max(m_accelVar[i], 1e-6);
    }
    m_rollVar = std::max(m_rollPitchVar[0], 1e-6);
    m_pitchVar = std::max(m_rollPitchVar[1], 1e-6);
  }

  const bool commanded = motor.commanded;
  const bool gravityLpOk = accelConfidence > 0.7 &&
                           gyroUnbiasedNorm < (m_config.stationary_gyro_thresh * 2.0) &&
                           (!commanded || accelOk);
  if (gravityLpOk)
  {
    const double alpha = EwmaAlpha(dt, m_config.gravity_lp_tau);
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
    const bool event = motor.commanded && (std::abs(motor.duty_dvdt) > m_config.dvdt_thresh ||
                                           accelLinXY > m_config.alin_thresh);
    const bool hasResponse =
        accelLinXY > m_config.forward_response_accel_thresh ||
        gyroUnbiasedNorm > m_config.forward_response_gyro_thresh;
    const bool gyroVeto = std::abs(gyroUnbiased[2]) > m_config.forward_gyro_veto_z ||
                          std::abs(gyroUnbiased[0]) > m_config.forward_gyro_veto_xy ||
                          std::abs(gyroUnbiased[1]) > m_config.forward_gyro_veto_xy;
    if (event && hasResponse && !gyroVeto)
    {
      const double alpha = EwmaAlpha(dt, m_config.ewma_tau);
      const double sign = motor.sign;
      if (std::abs(accelHoriz[0]) >= m_config.forward_deadband)
      {
        m_forwardScoreX = (1.0 - alpha) * m_forwardScoreX + alpha * sign * accelHoriz[0];
        m_forwardEnergyX = (1.0 - alpha) * m_forwardEnergyX + alpha * std::abs(accelHoriz[0]);
      }
      if (std::abs(accelHoriz[1]) >= m_config.forward_deadband)
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
      const bool dominantEligible = dominantAxis != 0 && dominantConf > m_config.forward_score_thresh &&
                                    std::abs(dominantAccel) >= m_config.forward_deadband;
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

        if (m_forwardSignConsistencyCount >= m_config.forward_sign_consistency_samples)
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

    if (m_forwardDominanceDuration >= m_config.forward_lock_seconds)
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
      RegisterMappingJump("forward-axis lock", output);
      output.info_messages.emplace_back("IMU forward axis locked");
    }
  }

  const double confX = std::abs(m_forwardScoreX) / (m_forwardEnergyX + 1e-6);
  const double confY = std::abs(m_forwardScoreY) / (m_forwardEnergyY + 1e-6);
  output.debug_values = {motor.duty_filt,
                         motor.duty_dvdt,
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

  output.forward_solved = m_forwardSolved;
  output.active_mapping = m_activeMapping;

  output.linear_acceleration = accel;
  output.angular_velocity = gyro;
  output.orientation = q;

  const double tempScale =
      m_hasTemperature ? 1.0 + m_config.temp_scale * std::abs(m_lastTemperature - m_temperatureAtCal)
                       : 1.0;
  Vec3 accelScaleNoise{0.0, 0.0, 0.0};
  Vec3 gyroScaleNoise{0.0, 0.0, 0.0};
  for (size_t i = 0; i < 3; ++i)
  {
    accelScaleNoise[i] = std::pow(m_config.accel_scale_noise * std::abs(accel[i]), 2.0) * tempScale;
    gyroScaleNoise[i] =
        std::pow(m_config.gyro_scale_noise * std::abs(gyroUnbiased[i]), 2.0) * tempScale;
  }

  const double rollVarPub = m_rollVar / std::max(accelConfidence, 0.1);
  const double pitchVarPub = m_pitchVar / std::max(accelConfidence, 0.1);
  m_yawVar += (m_gyroVar[2] + m_gyroBiasVar[2] + gyroScaleNoise[2]) * dt * dt;
  if (!m_isStationary || accelConfidence < 0.7)
    m_yawVar += m_config.yaw_inflate_factor * (1.0 - accelConfidence) * dt;

  output.orientation_covariance = {rollVarPub, 0.0, 0.0, 0.0, pitchVarPub, 0.0, 0.0, 0.0,
                                   m_yawVar};

  output.angular_velocity_covariance = {
      m_gyroVar[0] + m_gyroBiasVar[0] + gyroScaleNoise[0], 0.0, 0.0, 0.0,
      m_gyroVar[1] + m_gyroBiasVar[1] + gyroScaleNoise[1], 0.0, 0.0, 0.0,
      m_gyroVar[2] + m_gyroBiasVar[2] + gyroScaleNoise[2]};

  const double accelInflate = m_isStationary ? 1.0 : 1.0 / std::max(accelConfidence, 0.1);
  output.linear_acceleration_covariance = {
      accelInflate * (m_accelVar[0] + accelScaleNoise[0]), 0.0, 0.0, 0.0,
      accelInflate * (m_accelVar[1] + accelScaleNoise[1]), 0.0, 0.0, 0.0,
      accelInflate * (m_accelVar[2] + accelScaleNoise[2])};

  return output;
}
} // namespace OASIS::IMU
