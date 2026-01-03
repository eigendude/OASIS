/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/AccelCalibrator.h"
#include "imu/GyroBiasEstimator.h"
#include "imu/NoiseEstimator.h"

#include <array>
#include <cstdint>
#include <filesystem>

namespace OASIS::IMU
{
class Mpu6050ImuProcessor
{
public:
  struct ImuSample
  {
    // Linear acceleration in m/s^2 in the imu_link frame.
    std::array<double, 3> accel_mps2{0.0, 0.0, 0.0};

    // Accelerometer covariance in (m/s^2)^2 from the online 2nd-difference
    // estimator on raw counts, floored by 1 LSB quantization.
    AccelCalibrator::Mat3 accel_cov_mps2_2{};

    // Angular velocity in rad/s in the imu_link frame.
    std::array<double, 3> gyro_rads{0.0, 0.0, 0.0};

    // Gyroscope covariance in (rad/s)^2 from the online 2nd-difference
    // estimator on raw counts, floored by 1 LSB quantization.
    AccelCalibrator::Mat3 gyro_cov_rads2_2{};
  };

  struct ProcessedOutputs
  {
    // imu_raw: scaled sensor measurements in imu_link with noise covariances.
    ImuSample imu_raw{};

    // imu: calibrated stream with accelerometer correction applied.
    // Acceleration remains specific force (gravity is not removed) and gyro
    // bias is not subtracted.
    ImuSample imu{};

    // Calibration pipeline status for observability at the ROS layer.
    AccelCalibrator::UpdateStatus calibration_status{};
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

  void SetGravity(double gravityMps2)
  {
    m_gravity = gravityMps2;
    ConfigureCalibration(m_calibrationCachePath, m_calibrationFrameId);
  }
  double GetGravity() const { return m_gravity; }

  void SetAccelScale(double accelScale) { m_accelScale = accelScale; }
  double GetAccelScale() const { return m_accelScale; }

  void SetGyroScale(double gyroScale) { m_gyroScale = gyroScale; }
  double GetGyroScale() const { return m_gyroScale; }

  void ConfigureCalibration(const std::filesystem::path& cachePath, const std::string& frameId);
  bool LoadCachedCalibration();
  void SetCalibrationMode(bool enabled);
  bool HasCalibration() const { return m_accelCalibrator.HasSolution(); }
  const AccelCalibrator::Calibration& GetCalibration() const
  {
    return m_accelCalibrator.GetCalibration();
  }

  void Reset();

  ProcessedOutputs ProcessRaw(int16_t ax,
                              int16_t ay,
                              int16_t az,
                              int16_t gx,
                              int16_t gy,
                              int16_t gz,
                              double dt_s,
                              double temperature_c,
                              double timestamp_s);

private:
  // Configuration parameters
  double m_gravity{0.0};
  double m_accelScale{0.0};
  double m_gyroScale{0.0};

  // Estimators
  NoiseEstimator m_accelNoise;
  NoiseEstimator m_gyroNoise;
  GyroBiasEstimator m_gyroBiasEstimator;
  AccelCalibrator m_accelCalibrator;

  bool m_use_cached_noise{false};
  std::array<double, 3> m_cached_accel_noise_stddev{0.0, 0.0, 0.0};
  std::array<double, 3> m_cached_gyro_noise_stddev{0.0, 0.0, 0.0};
  AccelCalibrator::Mat3 m_cached_gyro_noise_cov{};

  std::filesystem::path m_calibrationCachePath;
  std::string m_calibrationFrameId{"imu_link"};
};
} // namespace OASIS::IMU
