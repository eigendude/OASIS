/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/Mpu6050ImuProcessor.h"

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

Mpu6050ImuProcessor::ProcessedSample Mpu6050ImuProcessor::ProcessRaw(
    int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) const
{
  ProcessedSample sample;

  sample.accel_mps2[0] = static_cast<double>(ax) * m_accelScale;
  sample.accel_mps2[1] = static_cast<double>(ay) * m_accelScale;
  sample.accel_mps2[2] = static_cast<double>(az) * m_accelScale;

  sample.gyro_rads[0] = static_cast<double>(gx) * m_gyroScale;
  sample.gyro_rads[1] = static_cast<double>(gy) * m_gyroScale;
  sample.gyro_rads[2] = static_cast<double>(gz) * m_gyroScale;

  return sample;
}
} // namespace OASIS::IMU
