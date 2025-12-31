/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

#include <algorithm>
#include <array>

using namespace OASIS::IMU;

void Mpu6050ImuProcessor::Reset()
{
  m_accelNoise.Reset();
  m_gyroNoise.Reset();
  m_gyroBiasEstimator.Reset();
}

Mpu6050ImuProcessor::ProcessedOutputs Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_s)
{
  ProcessedOutputs outputs{};

  const std::array<double, 3> accel_sensor_mps2{static_cast<double>(ax) * m_accelScale,
                                                static_cast<double>(ay) * m_accelScale,
                                                static_cast<double>(az) * m_accelScale};
  const std::array<double, 3> gyro_sensor_rads{static_cast<double>(gx) * m_gyroScale,
                                               static_cast<double>(gy) * m_gyroScale,
                                               static_cast<double>(gz) * m_gyroScale};

  const auto accel_sigma2_counts = m_accelNoise.Update(ax, ay, az, dt_s);
  const auto gyro_sigma2_counts = m_gyroNoise.Update(gx, gy, gz, dt_s);

  const double accel_floor = (m_accelScale * m_accelScale) / 12.0;
  const double gyro_floor = (m_gyroScale * m_gyroScale) / 12.0;

  std::array<double, 3> accel_var_sensor_mps2_2{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_var_sensor_rads2_2{0.0, 0.0, 0.0};

  for (size_t axis = 0; axis < accel_sigma2_counts.size(); ++axis)
  {
    const double accel_variance = accel_sigma2_counts[axis] * (m_accelScale * m_accelScale);
    const double gyro_variance = gyro_sigma2_counts[axis] * (m_gyroScale * m_gyroScale);

    accel_var_sensor_mps2_2[axis] = std::max(accel_variance, accel_floor);
    gyro_var_sensor_rads2_2[axis] = std::max(gyro_variance, gyro_floor);
  }

  outputs.imu_raw.accel_mps2 = accel_sensor_mps2;
  outputs.imu_raw.gyro_rads = gyro_sensor_rads;
  outputs.imu_raw.accel_var_mps2_2 = accel_var_sensor_mps2_2;
  outputs.imu_raw.gyro_var_rads2_2 = gyro_var_sensor_rads2_2;

  // TODO
  const std::array<double, 3> accel_body_mps2 = accel_sensor_mps2;
  const std::array<double, 3> gyro_body_rads = gyro_sensor_rads;

  m_gyroBiasEstimator.Update(gyro_body_rads, accel_body_mps2, m_gravity);
  outputs.gyro_bias_valid = m_gyroBiasEstimator.IsValid();

  std::array<double, 3> gyro_calibrated_rads = gyro_body_rads;
  if (outputs.gyro_bias_valid)
  {
    const auto& gyro_bias = m_gyroBiasEstimator.GetBias();
    for (size_t axis = 0; axis < gyro_calibrated_rads.size(); ++axis)
    {
      gyro_calibrated_rads[axis] -= gyro_bias[axis];
    }
  }

  outputs.imu.accel_mps2 = accel_body_mps2;
  outputs.imu.gyro_rads = gyro_calibrated_rads;

  // TODO
  outputs.imu.accel_var_mps2_2 = outputs.imu_raw.accel_var_mps2_2;
  outputs.imu.gyro_var_rads2_2 = outputs.imu_raw.gyro_var_rads2_2;

  return outputs;
}
