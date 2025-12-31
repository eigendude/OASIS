/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/GyroBiasEstimator.h"
#include "imu/NoiseEstimator.h"

#include <array>
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
  };

  struct AxisRemap
  {
    // map[imu_axis] gives the source sensor axis index that feeds imu_axis.
    std::array<size_t, 3> map{0, 1, 2};

    // sign[imu_axis] flips the mapped source axis so imu axes follow ROS
    // conventions.
    std::array<int, 3> sign{1, 1, 1};
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
  // Configuration parameters
  double m_gravity{0.0};
  double m_accelScale{0.0};
  double m_gyroScale{0.0};

  // Estimators
  NoiseEstimator m_accelNoise;
  NoiseEstimator m_gyroNoise;
  GyroBiasEstimator m_gyroBiasEstimator;
};
} // namespace OASIS::IMU
