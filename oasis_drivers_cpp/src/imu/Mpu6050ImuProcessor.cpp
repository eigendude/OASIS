/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include "imu/ImuMath.h"

#include <cmath>

namespace
{
constexpr double kG = 9.80665;
constexpr std::size_t kBootTrimStationarySamples = 50;
constexpr double kEps = 1e-9;
constexpr double kUpAlpha = 0.1;
} // namespace

using namespace OASIS::IMU;

Mpu6050ImuProcessor::Mpu6050ImuProcessor() = default;

void Mpu6050ImuProcessor::SetAccelScale(double accelScale)
{
  m_accelScale = accelScale;
}

double Mpu6050ImuProcessor::GetAccelScale() const
{
  return m_accelScale;
}

void Mpu6050ImuProcessor::SetGyroScale(double gyroScale)
{
  m_gyroScale = gyroScale;
}

Mpu6050ImuProcessor::ProcessedSample Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_seconds)
{
  ProcessedSample sample{};

  sample.accel_raw_mps2 = {static_cast<double>(ax) * m_accelScale,
                           static_cast<double>(ay) * m_accelScale,
                           static_cast<double>(az) * m_accelScale};
  sample.gyro_rads = {static_cast<double>(gx) * m_gyroScale, static_cast<double>(gy) * m_gyroScale,
                      static_cast<double>(gz) * m_gyroScale};

  sample.diag = m_accelCalibrator.Update(sample.accel_raw_mps2, sample.gyro_rads, dt_seconds);

  if (!m_boot_accel_scale_applied && sample.diag.stationary)
  {
    const double raw_norm_lsb = std::sqrt(static_cast<double>(ax) * static_cast<double>(ax) +
                                          static_cast<double>(ay) * static_cast<double>(ay) +
                                          static_cast<double>(az) * static_cast<double>(az));

    ++m_boot_stationary_samples;
    m_boot_stationary_mean +=
        (raw_norm_lsb - m_boot_stationary_mean) / static_cast<double>(m_boot_stationary_samples);

    if (m_boot_stationary_samples >= kBootTrimStationarySamples && m_boot_stationary_mean > kEps)
    {
      const double lsb_per_g = m_boot_stationary_mean;
      const double new_accel_scale = kG / lsb_per_g;

      m_accelScale = new_accel_scale;
      m_boot_accel_scale_applied = true;
      m_accelCalibrator.ResetUniformScaleInitialization();

      sample.accel_raw_mps2 = {static_cast<double>(ax) * m_accelScale,
                               static_cast<double>(ay) * m_accelScale,
                               static_cast<double>(az) * m_accelScale};
      sample.diag = m_accelCalibrator.Update(sample.accel_raw_mps2, sample.gyro_rads, dt_seconds);

      sample.boot_accel_scale_applied = true;
      sample.boot_lsb_per_g = lsb_per_g;
      sample.boot_accel_scale = new_accel_scale;
    }
  }

  sample.accel_mps2 = {(sample.accel_raw_mps2[0] - sample.diag.bias_mps2[0]) / sample.diag.scale[0],
                       (sample.accel_raw_mps2[1] - sample.diag.bias_mps2[1]) / sample.diag.scale[1],
                       (sample.accel_raw_mps2[2] - sample.diag.bias_mps2[2]) /
                           sample.diag.scale[2]};

  if (sample.diag.stationary)
  {
    const double accel_norm = Math::Norm(sample.accel_mps2);
    if (accel_norm > kEps)
    {
      const std::array<double, 3> u_meas = Math::Scale(sample.accel_mps2, 1.0 / accel_norm);
      if (!m_u_hat_valid)
      {
        m_u_hat = u_meas;
        m_u_hat_valid = true;
      }
      else
      {
        m_u_hat = Math::Normalize(
            Math::Add(Math::Scale(m_u_hat, 1.0 - kUpAlpha), Math::Scale(u_meas, kUpAlpha)));
      }
    }
  }

  sample.u_hat = m_u_hat;
  sample.u_hat_valid = m_u_hat_valid;
  sample.forward_diag = m_forwardAxisLearner.Update(sample.accel_mps2, sample.gyro_rads, dt_seconds,
                                                    m_u_hat, m_u_hat_valid);
  sample.f_hat_unsigned = m_forwardAxisLearner.GetForwardAxisUnsigned();
  sample.f_hat_locked = m_forwardAxisLearner.IsLocked();

  return sample;
}
