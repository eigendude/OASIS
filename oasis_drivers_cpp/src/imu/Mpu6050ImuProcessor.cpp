/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

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

  sample.accel_mps2 = {(sample.accel_raw_mps2[0] - sample.diag.bias_mps2[0]) / sample.diag.scale[0],
                       (sample.accel_raw_mps2[1] - sample.diag.bias_mps2[1]) / sample.diag.scale[1],
                       (sample.accel_raw_mps2[2] - sample.diag.bias_mps2[2]) /
                           sample.diag.scale[2]};

  return sample;
}
