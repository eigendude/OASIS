/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  struct ProcessedSample
  {
    // Raw accelerometer sample in m/s^2, converted from sensor counts.
    std::array<double, 3> accel_raw_mps2{0.0, 0.0, 0.0};

    // Per-axis accelerometer variance in (m/s^2)^2 from the online
    // 2nd-difference estimator on raw counts, floored by 1 LSB quantization.
    std::array<double, 3> accel_var_mps2_2{0.0, 0.0, 0.0};

    // Gyroscope sample in rad/s, converted from sensor counts.
    std::array<double, 3> gyro_rads{0.0, 0.0, 0.0};

    // Per-axis gyroscope variance in (rad/s)^2 from the online
    // 2nd-difference estimator on raw counts, floored by 1 LSB quantization.
    std::array<double, 3> gyro_var_rads2_2{0.0, 0.0, 0.0};
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
      int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_s);

private:
  /**
   * Jitter-aware 2nd-difference estimator for i.i.d. measurement noise.
   *
   * Given samples x0, x1, x2 with time steps h2 = t1 - t0 and h1 = t2 - t1,
   * the non-uniform derivative residual is:
   *
   *   d = (x2 - x1) / h1 - (x1 - x0) / h2.
   *
   * For independent noise with variance sigma^2, Var(d) = K * sigma^2 with:
   *
   *   K = 2 * (1 / h1^2 + 1 / h2^2 + 1 / (h1 * h2)).
   *
   * So the per-sample estimate is:
   *
   *   sigma^2 = d^2 / K.
   *
   * For uniform dt this reduces to sigma^2 = (d2^2) / 6 with
   * d2 = x2 - 2 * x1 + x0.
   */
  class NoiseEstimator
  {
  public:
    void Reset();
    std::array<double, 3> Update(int16_t x, int16_t y, int16_t z, double dt_s, bool is_stationary);

  private:
    static constexpr size_t kWindowSize = 16;

    std::array<int32_t, 3> m_prev1_counts{0, 0, 0};
    std::array<int32_t, 3> m_prev2_counts{0, 0, 0};
    std::array<std::array<double, kWindowSize>, 3> m_sigma2_window{};
    std::array<double, 3> m_sigma2_sum{0.0, 0.0, 0.0};
    double m_prev_dt_s{0.0};
    size_t m_sigma2_index{0};
    size_t m_sigma2_count{0};
    int m_samples{0};
  };

  double m_gravity{0.0};
  double m_accelScale{0.0};
  double m_gyroScale{0.0};
  NoiseEstimator m_accelNoise;
  NoiseEstimator m_gyroNoise;
};
} // namespace OASIS::IMU
