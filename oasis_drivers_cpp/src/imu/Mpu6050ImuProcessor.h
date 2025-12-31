/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstdint>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  struct ProcessedSample
  {
    // Raw accelerometer sample in m/s^2, converted from sensor counts
    std::array<double, 3> accel_raw_mps2{0.0, 0.0, 0.0};

    // Gyroscope sample in rad/s, converted from sensor counts
    std::array<double, 3> gyro_rads{0.0, 0.0, 0.0};
  };

  Mpu6050ImuProcessor();

  void SetAccelScale(double accelScale);
  double GetAccelScale() const;
  void SetGyroScale(double gyroScale);

  ProcessedSample ProcessRaw(
      int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

private:
  double m_accelScale{0.0};
  double m_gyroScale{0.0};
};
} // namespace OASIS::IMU
