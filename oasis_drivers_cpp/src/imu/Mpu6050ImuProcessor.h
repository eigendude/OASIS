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

    // Per-axis accelerometer variance in (m/s^2)^2, combining quantization
    // noise with an online analog noise estimate from raw counts.
    std::array<double, 3> accel_variance_mps2{0.0, 0.0, 0.0};

    // Gyroscope sample in rad/s, converted from sensor counts
    std::array<double, 3> gyro_rads{0.0, 0.0, 0.0};

    // Per-axis gyroscope variance in (rad/s)^2, combining quantization
    // noise with an online analog noise estimate from raw counts.
    std::array<double, 3> gyro_variance_rads{0.0, 0.0, 0.0};
  };

  Mpu6050ImuProcessor() = default;

  void SetGravity(double gravityMps2) { m_gravity = gravityMps2; }
  double GetGravity() const { return m_gravity; }

  void SetAccelScale(double accelScale) { m_accelScale = accelScale; }
  double GetAccelScale() const { return m_accelScale; }

  void SetGyroScale(double gyroScale) { m_gyroScale = gyroScale; }
  double GetGyroScale() const { return m_gyroScale; }

  void Reset();

  ProcessedSample ProcessRaw(
      int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

private:
  class NoiseEstimator
  {
  public:
    void Reset();
    std::array<double, 3> Update(int16_t x, int16_t y, int16_t z);

  private:
    std::array<int32_t, 3> m_prev1_counts{0, 0, 0};
    std::array<int32_t, 3> m_prev2_counts{0, 0, 0};
    std::array<double, 3> m_sigma2_counts{0.0, 0.0, 0.0};
    int m_samples{0};
  };

  double m_gravity{0.0};
  double m_accelScale{0.0};
  double m_gyroScale{0.0};
  NoiseEstimator m_accelNoise;
  NoiseEstimator m_gyroNoise;
};
} // namespace OASIS::IMU
