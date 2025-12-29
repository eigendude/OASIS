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
    //! Bias-corrected acceleration, meters per second^2 (near 9.81 in
    //! magnitude at rest).
    std::array<double, 3> accel_mps2{};
    //! Raw scaled acceleration before bias correction, meters per second^2
    //! (sensor output range).
    std::array<double, 3> accel_raw_mps2{};
    //! Scaled angular rate, radians per second (near 0 when stationary).
    std::array<double, 3> gyro_rads{};
    //! Accelerometer calibrator diagnostics for the current sample window
    //! (computed during Update()).
    AccelCalibrator::Diagnostics bias_diag{};
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
  AccelCalibrator m_biasEstimator;
};
} // namespace OASIS::IMU
