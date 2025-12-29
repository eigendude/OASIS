/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/AccelBiasEstimator.h"

#include <array>
#include <cstdint>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  struct ProcessedSample
  {
    std::array<double, 3> accel_mps2{};
    std::array<double, 3> accel_raw_mps2{};
    std::array<double, 3> gyro_rads{};
    AccelBiasEstimator::Diagnostics bias_diag{};
  };

  Mpu6050ImuProcessor();
  ~Mpu6050ImuProcessor();

  void SetAccelScale(double accelScale);
  void SetGyroScale(double gyroScale);
  ProcessedSample ProcessRaw(
      int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_seconds);

private:
  double m_accelScale = 0.0;
  double m_gyroScale = 0.0;
  AccelBiasEstimator m_biasEstimator;
};
} // namespace OASIS::IMU
