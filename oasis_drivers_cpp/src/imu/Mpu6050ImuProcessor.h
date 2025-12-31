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
  struct ImuSample
  {
    // Linear acceleration in m/s^2 in the imu_link frame.
    std::array<double, 3> accel_mps2{0.0, 0.0, 0.0};

    // Per-axis accelerometer variance in (m/s^2)^2 from the online
    // 2nd-difference estimator on raw counts, floored by 1 LSB quantization.
    std::array<double, 3> accel_var_mps2_2{0.0, 0.0, 0.0};

    // Angular velocity in rad/s in the imu_link frame.
    std::array<double, 3> gyro_rads{0.0, 0.0, 0.0};

    // Per-axis gyroscope variance in (rad/s)^2 from the online
    // 2nd-difference estimator on raw counts, floored by 1 LSB quantization.
    std::array<double, 3> gyro_var_rads2_2{0.0, 0.0, 0.0};
  };

  struct ProcessedOutputs
  {
    // imu_raw: scaled sensor measurements in imu_link with noise covariances.
    ImuSample imu_raw{};

    // imu: calibrated stream for ORB-SLAM3 with gyro bias subtraction.
    // Acceleration remains specific force (gravity is not removed).
    ImuSample imu{};

    // True once the gyro bias estimate has reached its minimum sample
    // threshold.
    bool gyro_bias_valid{false};
  };

  Mpu6050ImuProcessor() = default;

  void SetGravity(double gravityMps2) { m_gravity = gravityMps2; }
  double GetGravity() const { return m_gravity; }

  void SetAccelScale(double accelScale) { m_accelScale = accelScale; }
  double GetAccelScale() const { return m_accelScale; }

  void SetGyroScale(double gyroScale) { m_gyroScale = gyroScale; }
  double GetGyroScale() const { return m_gyroScale; }

  void Reset();

  ProcessedOutputs ProcessRaw(
      int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double dt_s);

private:
  class GyroBiasEstimator
  {
  public:
    void Reset();
    void Update(const std::array<double, 3>& gyro_rads,
                const std::array<double, 3>& accel_mps2,
                double gravity_mps2);
    const std::array<double, 3>& GetBias() const { return m_bias_rads; }
    bool IsValid() const { return m_valid; }

  private:
    std::array<double, 3> m_bias_rads{0.0, 0.0, 0.0};
    int m_stationary_samples{0};
    bool m_valid{false};
  };

  /**
   * Jitter-aware 2nd-difference estimator for i.i.d. measurement noise.
   *
   * Given samples x0, x1, x2 with time steps h1, h2, the non-uniform
   * second-difference residual is:
   *
   *   d2 = x2 - x1 * (h1 + h2) / h1 + x0 * h2 / h1.
   *
   * Define a = h2 / h1 and b = 1 + a. For independent noise with variance
   * sigma^2, Var(d2) = (a^2 + b^2 + 1) * sigma^2, so K(h1, h2) is
   *
   *   K = a^2 + b^2 + 1
   *
   * and
   *
   *   sigma^2 = d2^2 / (a^2 + b^2 + 1).
   *
   * If h1 is tiny, fall back to the uniform-grid formula:
   *
   *   d2 = x2 - 2 * x1 + x0,  sigma^2 = d2^2 / 6.
   */
  class NoiseEstimator
  {
  public:
    void Reset();
    std::array<double, 3> Update(int16_t x, int16_t y, int16_t z, double dt_s);

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
  GyroBiasEstimator m_gyroBiasEstimator;
};
} // namespace OASIS::IMU
